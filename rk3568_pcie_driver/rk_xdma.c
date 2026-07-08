// SPDX-License-Identifier: GPL-2.0
/*
 * RK3568 PCIe Root Complex driver for a Xilinx XDMA FPGA endpoint.
 *
 * Data model:
 *   - Small control/status messages: userspace reads/writes FPGA BAR space.
 *   - Video stream H2C: userspace fills one coherent DMA buffer, then submits
 *     a single XDMA descriptor to copy host memory into FPGA AXI space.
 *
 * This is intentionally small and board-facing. For production video, extend
 * h2c_submit() into a queued, interrupt-driven descriptor ring.
 */

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/poll.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#include "rk_xdma_ioctl.h"

#define DRV_NAME "rk3568-xdma"
#define RK_XDMA_MAX_BARS 6

/*
 * XDMA register map for the standard Xilinx XDMA IP AXI-MM H2C engine.
 * If the FPGA build changes BAR placement or IP configuration, adjust the
 * module parameters below instead of touching user code.
 */
#define XDMA_H2C_BASE(ch)          (0x0000 + (ch) * 0x100)
#define XDMA_ENGINE_CONTROL        0x04
#define XDMA_ENGINE_STATUS         0x40
#define XDMA_COMPLETED_DESC        0x48
#define XDMA_FIRST_DESC_LO         0x80
#define XDMA_FIRST_DESC_HI         0x84
#define XDMA_FIRST_DESC_ADJACENT   0x88

#define XDMA_CTRL_RUN_STOP         BIT(0)
#define XDMA_DESC_MAGIC            0xad4b0000u
#define XDMA_DESC_STOP             BIT(0)
#define XDMA_DESC_EOP              BIT(4)

static unsigned int xdma_bar = 0;
module_param(xdma_bar, uint, 0444);
MODULE_PARM_DESC(xdma_bar, "BAR containing XDMA engine registers");

static unsigned int user_bar = 2;
module_param(user_bar, uint, 0444);
MODULE_PARM_DESC(user_bar, "BAR containing FPGA user/control registers");

static unsigned int h2c_channel = 0;
module_param(h2c_channel, uint, 0444);
MODULE_PARM_DESC(h2c_channel, "XDMA H2C channel index");

static unsigned int default_timeout_ms = 1000;
module_param(default_timeout_ms, uint, 0644);
MODULE_PARM_DESC(default_timeout_ms, "Default DMA completion timeout in ms");

static bool force_dma32 = true;
module_param(force_dma32, bool, 0444);
MODULE_PARM_DESC(force_dma32, "Force 32-bit coherent DMA addresses for RK PCIe bring-up");

struct xdma_desc {
	__le32 control;
	__le32 bytes;
	__le32 src_addr_lo;
	__le32 src_addr_hi;
	__le32 dst_addr_lo;
	__le32 dst_addr_hi;
	__le32 next_lo;
	__le32 next_hi;
} __packed;

struct rk_xdma_dev {
	struct pci_dev *pdev;
	void __iomem *bar[RK_XDMA_MAX_BARS];
	resource_size_t bar_len[RK_XDMA_MAX_BARS];

	struct class *class;
	struct device *device;
	struct cdev cdev;
	dev_t devt;
	struct mutex lock;

	void *buf_virt;
	dma_addr_t buf_dma;
	size_t buf_size;

	struct xdma_desc *desc_virt;
	dma_addr_t desc_dma;

	u32 last_h2c_status;
	u32 last_h2c_completed;
};

static struct rk_xdma_dev *file_to_rxd(struct file *filp)
{
	return filp->private_data;
}

static void __iomem *bar_ptr(struct rk_xdma_dev *rxd, u32 bar, u64 off, u32 width)
{
	if (bar >= RK_XDMA_MAX_BARS || !rxd->bar[bar])
		return NULL;
	if (width != 1 && width != 2 && width != 4)
		return NULL;
	if (off > rxd->bar_len[bar] || width > rxd->bar_len[bar] - off)
		return NULL;
	if (off & (width - 1))
		return NULL;

	return rxd->bar[bar] + off;
}

static void xdma_engine_write(struct rk_xdma_dev *rxd, u32 reg, u32 val)
{
	iowrite32(val, rxd->bar[xdma_bar] + XDMA_H2C_BASE(h2c_channel) + reg);
}

static u32 xdma_engine_read(struct rk_xdma_dev *rxd, u32 reg)
{
	return ioread32(rxd->bar[xdma_bar] + XDMA_H2C_BASE(h2c_channel) + reg);
}

static void xdma_log_h2c_status(struct rk_xdma_dev *rxd, const char *tag)
{
	rxd->last_h2c_status = xdma_engine_read(rxd, XDMA_ENGINE_STATUS);
	rxd->last_h2c_completed = xdma_engine_read(rxd, XDMA_COMPLETED_DESC);
	dev_info(&rxd->pdev->dev, "%s H2C%u status=0x%08x completed=%u\n",
		 tag, h2c_channel, rxd->last_h2c_status,
		 rxd->last_h2c_completed);
}

static int h2c_submit(struct rk_xdma_dev *rxd, u64 bytes, u64 fpga_addr,
		      u32 timeout_ms)
{
	ktime_t deadline;
	u32 before;

	if (!rxd->bar[xdma_bar])
		return -ENODEV;
	if (!rxd->buf_virt || !rxd->desc_virt)
		return -ENOMEM;
	if (!bytes || bytes > rxd->buf_size || bytes > U32_MAX)
		return -EINVAL;

	memset(rxd->desc_virt, 0, sizeof(*rxd->desc_virt));
	rxd->desc_virt->control = cpu_to_le32(XDMA_DESC_MAGIC | XDMA_DESC_STOP |
					      XDMA_DESC_EOP);
	rxd->desc_virt->bytes = cpu_to_le32((u32)bytes);
	rxd->desc_virt->src_addr_lo = cpu_to_le32(lower_32_bits(rxd->buf_dma));
	rxd->desc_virt->src_addr_hi = cpu_to_le32(upper_32_bits(rxd->buf_dma));
	rxd->desc_virt->dst_addr_lo = cpu_to_le32(lower_32_bits(fpga_addr));
	rxd->desc_virt->dst_addr_hi = cpu_to_le32(upper_32_bits(fpga_addr));

	dma_wmb();

	xdma_engine_write(rxd, XDMA_ENGINE_CONTROL, 0);
	udelay(10);
	xdma_log_h2c_status(rxd, "before-submit");

	before = xdma_engine_read(rxd, XDMA_COMPLETED_DESC);
	xdma_engine_write(rxd, XDMA_FIRST_DESC_LO, lower_32_bits(rxd->desc_dma));
	xdma_engine_write(rxd, XDMA_FIRST_DESC_HI, upper_32_bits(rxd->desc_dma));
	xdma_engine_write(rxd, XDMA_FIRST_DESC_ADJACENT, 0);
	xdma_engine_write(rxd, XDMA_ENGINE_CONTROL, XDMA_CTRL_RUN_STOP);

	if (!timeout_ms)
		timeout_ms = default_timeout_ms;

	deadline = ktime_add_ms(ktime_get(), timeout_ms);
	do {
		rxd->last_h2c_status = xdma_engine_read(rxd, XDMA_ENGINE_STATUS);
		rxd->last_h2c_completed = xdma_engine_read(rxd, XDMA_COMPLETED_DESC);
		if (rxd->last_h2c_completed != before)
			return 0;
		usleep_range(50, 100);
	} while (ktime_before(ktime_get(), deadline));

	dev_err(&rxd->pdev->dev, "H2C timeout status=0x%08x completed=%u before=%u\n",
		rxd->last_h2c_status, rxd->last_h2c_completed, before);
	return -ETIMEDOUT;
}

static void rk_xdma_log_pcie_info(struct pci_dev *pdev)
{
	u16 lnksta = 0;
	int ret;

	dev_info(&pdev->dev, "PCI ID %04x:%04x subsystem %04x:%04x class=0x%06x irq=%d\n",
		 pdev->vendor, pdev->device, pdev->subsystem_vendor,
		 pdev->subsystem_device, pdev->class, pdev->irq);

	ret = pcie_capability_read_word(pdev, PCI_EXP_LNKSTA, &lnksta);
	if (!ret)
		dev_info(&pdev->dev, "PCIe link status raw=0x%04x speed_code=%u width=x%u\n",
			 lnksta, lnksta & PCI_EXP_LNKSTA_CLS,
			 (lnksta & PCI_EXP_LNKSTA_NLW) >> 4);
}

static int rk_xdma_open(struct inode *inode, struct file *filp)
{
	struct rk_xdma_dev *rxd = container_of(inode->i_cdev, struct rk_xdma_dev,
					      cdev);
	filp->private_data = rxd;
	return 0;
}

static ssize_t rk_xdma_write(struct file *filp, const char __user *ubuf,
			     size_t len, loff_t *ppos)
{
	struct rk_xdma_dev *rxd = file_to_rxd(filp);
	ssize_t ret;

	mutex_lock(&rxd->lock);
	if (!rxd->buf_virt) {
		ret = -ENOMEM;
		goto out;
	}
	if (*ppos < 0 || *ppos >= rxd->buf_size) {
		ret = -EINVAL;
		goto out;
	}
	len = min_t(size_t, len, rxd->buf_size - *ppos);
	if (copy_from_user(rxd->buf_virt + *ppos, ubuf, len)) {
		ret = -EFAULT;
		goto out;
	}
	*ppos += len;
	ret = len;
out:
	mutex_unlock(&rxd->lock);
	return ret;
}

static int rk_xdma_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct rk_xdma_dev *rxd = file_to_rxd(filp);
	unsigned long len = vma->vm_end - vma->vm_start;
	int ret;

	mutex_lock(&rxd->lock);
	if (!rxd->buf_virt || len > rxd->buf_size) {
		ret = -EINVAL;
		goto out;
	}
	ret = dma_mmap_coherent(&rxd->pdev->dev, vma, rxd->buf_virt,
				rxd->buf_dma, rxd->buf_size);
out:
	mutex_unlock(&rxd->lock);
	return ret;
}

static long rk_xdma_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct rk_xdma_dev *rxd = file_to_rxd(filp);
	void __iomem *addr;
	long ret = 0;

	mutex_lock(&rxd->lock);

	switch (cmd) {
	case RK_XDMA_IOC_BAR_READ: {
		struct rk_xdma_bar_io io;

		if (copy_from_user(&io, (void __user *)arg, sizeof(io))) {
			ret = -EFAULT;
			break;
		}
		addr = bar_ptr(rxd, io.bar, io.offset, io.width);
		if (!addr) {
			ret = -EINVAL;
			break;
		}
		if (io.width == 1)
			io.value = ioread8(addr);
		else if (io.width == 2)
			io.value = ioread16(addr);
		else
			io.value = ioread32(addr);
		if (copy_to_user((void __user *)arg, &io, sizeof(io)))
			ret = -EFAULT;
		break;
	}
	case RK_XDMA_IOC_BAR_WRITE: {
		struct rk_xdma_bar_io io;

		if (copy_from_user(&io, (void __user *)arg, sizeof(io))) {
			ret = -EFAULT;
			break;
		}
		addr = bar_ptr(rxd, io.bar, io.offset, io.width);
		if (!addr) {
			ret = -EINVAL;
			break;
		}
		if (io.width == 1)
			iowrite8(io.value, addr);
		else if (io.width == 2)
			iowrite16(io.value, addr);
		else
			iowrite32(io.value, addr);
		break;
	}
	case RK_XDMA_IOC_DMA_ALLOC: {
		struct rk_xdma_dma_alloc alloc;

		if (copy_from_user(&alloc, (void __user *)arg, sizeof(alloc))) {
			ret = -EFAULT;
			break;
		}
		if (!alloc.size || alloc.size > SZ_256M) {
			ret = -EINVAL;
			break;
		}
		if (rxd->buf_virt)
			dma_free_coherent(&rxd->pdev->dev, rxd->buf_size,
					  rxd->buf_virt, rxd->buf_dma);
		rxd->buf_size = PAGE_ALIGN(alloc.size);
		rxd->buf_virt = dma_alloc_coherent(&rxd->pdev->dev, rxd->buf_size,
						   &rxd->buf_dma, GFP_KERNEL);
		if (!rxd->buf_virt) {
			rxd->buf_size = 0;
			ret = -ENOMEM;
		}
		break;
	}
	case RK_XDMA_IOC_DMA_FREE:
		if (rxd->buf_virt) {
			dma_free_coherent(&rxd->pdev->dev, rxd->buf_size,
					  rxd->buf_virt, rxd->buf_dma);
			rxd->buf_virt = NULL;
			rxd->buf_dma = 0;
			rxd->buf_size = 0;
		}
		break;
	case RK_XDMA_IOC_H2C_SUBMIT: {
		struct rk_xdma_h2c h2c;

		if (copy_from_user(&h2c, (void __user *)arg, sizeof(h2c))) {
			ret = -EFAULT;
			break;
		}
		ret = h2c_submit(rxd, h2c.bytes, h2c.fpga_addr, h2c.timeout_ms);
		break;
	}
	case RK_XDMA_IOC_STATUS: {
		struct rk_xdma_status st = {
			.h2c_status = rxd->last_h2c_status,
			.h2c_completed_desc = rxd->last_h2c_completed,
		};

		if (rxd->bar[xdma_bar]) {
			st.h2c_status = xdma_engine_read(rxd, XDMA_ENGINE_STATUS);
			st.h2c_completed_desc =
				xdma_engine_read(rxd, XDMA_COMPLETED_DESC);
		}
		if (copy_to_user((void __user *)arg, &st, sizeof(st)))
			ret = -EFAULT;
		break;
	}
	default:
		ret = -ENOTTY;
		break;
	}

	mutex_unlock(&rxd->lock);
	return ret;
}

static const struct file_operations rk_xdma_fops = {
	.owner = THIS_MODULE,
	.open = rk_xdma_open,
	.write = rk_xdma_write,
	.mmap = rk_xdma_mmap,
	.unlocked_ioctl = rk_xdma_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = rk_xdma_ioctl,
#endif
};

static int rk_xdma_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct rk_xdma_dev *rxd;
	void __iomem * const *iomap_table;
	int bars;
	int bar, ret;

	rxd = devm_kzalloc(&pdev->dev, sizeof(*rxd), GFP_KERNEL);
	if (!rxd)
		return -ENOMEM;

	rxd->pdev = pdev;
	mutex_init(&rxd->lock);
	pci_set_drvdata(pdev, rxd);

	ret = pcim_enable_device(pdev);
	if (ret)
		return ret;

	rk_xdma_log_pcie_info(pdev);

	if (force_dma32)
		ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	else {
		ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
		if (ret)
			ret = dma_set_mask_and_coherent(&pdev->dev,
							DMA_BIT_MASK(32));
	}
	if (ret)
		return ret;
	dev_info(&pdev->dev, "using %s coherent DMA mask\n",
		 force_dma32 ? "32-bit forced" : "best available");

	pci_set_master(pdev);

	bars = pci_select_bars(pdev, IORESOURCE_MEM);
	dev_info(&pdev->dev, "memory BAR mask=0x%x xdma_bar=%u user_bar=%u h2c_channel=%u\n",
		 bars, xdma_bar, user_bar, h2c_channel);
	ret = pcim_iomap_regions(pdev, bars, DRV_NAME);
	if (ret)
		return ret;

	iomap_table = pcim_iomap_table(pdev);
	if (!iomap_table)
		return -ENOMEM;

	for (bar = 0; bar < RK_XDMA_MAX_BARS; bar++) {
		if (!(pci_resource_flags(pdev, bar) & IORESOURCE_MEM))
			continue;
		rxd->bar_len[bar] = pci_resource_len(pdev, bar);
		rxd->bar[bar] = iomap_table[bar];
		if (!rxd->bar[bar]) {
			dev_err(&pdev->dev, "failed to map BAR%d\n", bar);
			return -ENOMEM;
		}
		dev_info(&pdev->dev, "BAR%d mapped start=%pa len=%pa flags=0x%lx\n",
			 bar, &pdev->resource[bar].start, &rxd->bar_len[bar],
			 pci_resource_flags(pdev, bar));
	}

	if (xdma_bar >= RK_XDMA_MAX_BARS || !rxd->bar[xdma_bar]) {
		dev_err(&pdev->dev, "xdma_bar=%u is not mapped\n", xdma_bar);
		return -EINVAL;
	}
	if (user_bar >= RK_XDMA_MAX_BARS || !rxd->bar[user_bar])
		dev_warn(&pdev->dev, "user_bar=%u is not mapped\n", user_bar);

	rxd->desc_virt = dma_alloc_coherent(&pdev->dev, sizeof(*rxd->desc_virt),
					    &rxd->desc_dma, GFP_KERNEL);
	if (!rxd->desc_virt)
		return -ENOMEM;
	dev_info(&pdev->dev, "descriptor dma=%pad size=%zu\n", &rxd->desc_dma,
		 sizeof(*rxd->desc_virt));

	ret = alloc_chrdev_region(&rxd->devt, 0, 1, DRV_NAME);
	if (ret)
		goto err_desc;

	cdev_init(&rxd->cdev, &rk_xdma_fops);
	rxd->cdev.owner = THIS_MODULE;
	ret = cdev_add(&rxd->cdev, rxd->devt, 1);
	if (ret)
		goto err_chrdev;

	rxd->class = class_create(THIS_MODULE, DRV_NAME);
	if (IS_ERR(rxd->class)) {
		ret = PTR_ERR(rxd->class);
		goto err_cdev;
	}

	rxd->device = device_create(rxd->class, &pdev->dev, rxd->devt, NULL,
				    "rk_xdma%d", MINOR(rxd->devt));
	if (IS_ERR(rxd->device)) {
		ret = PTR_ERR(rxd->device);
		goto err_class;
	}

	dev_info(&pdev->dev, "RK3568 XDMA endpoint ready: /dev/rk_xdma%d\n",
		 MINOR(rxd->devt));
	return 0;

err_class:
	class_destroy(rxd->class);
err_cdev:
	cdev_del(&rxd->cdev);
err_chrdev:
	unregister_chrdev_region(rxd->devt, 1);
err_desc:
	dma_free_coherent(&pdev->dev, sizeof(*rxd->desc_virt), rxd->desc_virt,
			  rxd->desc_dma);
	return ret;
}

static void rk_xdma_remove(struct pci_dev *pdev)
{
	struct rk_xdma_dev *rxd = pci_get_drvdata(pdev);

	device_destroy(rxd->class, rxd->devt);
	class_destroy(rxd->class);
	cdev_del(&rxd->cdev);
	unregister_chrdev_region(rxd->devt, 1);

	if (rxd->buf_virt)
		dma_free_coherent(&pdev->dev, rxd->buf_size, rxd->buf_virt,
				  rxd->buf_dma);
	dma_free_coherent(&pdev->dev, sizeof(*rxd->desc_virt), rxd->desc_virt,
			  rxd->desc_dma);
}

static const struct pci_device_id rk_xdma_ids[] = {
	{ PCI_DEVICE(0x10ee, PCI_ANY_ID) },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, rk_xdma_ids);

static struct pci_driver rk_xdma_pci_driver = {
	.name = DRV_NAME,
	.id_table = rk_xdma_ids,
	.probe = rk_xdma_probe,
	.remove = rk_xdma_remove,
};

module_pci_driver(rk_xdma_pci_driver);

MODULE_AUTHOR("Codex");
MODULE_DESCRIPTION("RK3568 ARM PCIe host driver for Xilinx XDMA FPGA endpoint");
MODULE_LICENSE("GPL");
