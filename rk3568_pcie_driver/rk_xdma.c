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
#define DRV_VERSION "2026-07-10-xdma-start-seq-v5"
#define RK_XDMA_MAX_BARS 6

/*
 * XDMA register map for the standard Xilinx XDMA IP AXI-MM H2C engine.
 * If the FPGA build changes BAR placement or IP configuration, adjust the
 * module parameters below instead of touching user code.
 */
#define XDMA_H2C_BASE(ch)          (0x0000 + (ch) * 0x100)
#define XDMA_ENGINE_CONTROL        0x04
#define XDMA_ENGINE_CONTROL_W1S    0x08
#define XDMA_ENGINE_CONTROL_W1C    0x0c
#define XDMA_ENGINE_STATUS         0x40
#define XDMA_ENGINE_STATUS_RC      0x44
#define XDMA_COMPLETED_DESC        0x48
#define XDMA_ENGINE_ALIGNMENTS     0x4c
#define XDMA_FIRST_DESC_LO         0x80
#define XDMA_FIRST_DESC_HI         0x84
#define XDMA_FIRST_DESC_ADJACENT   0x88

#define XDMA_CTRL_RUN_STOP         BIT(0)
#define XDMA_CTRL_IE_DESC_STOPPED  BIT(1)
#define XDMA_CTRL_IE_DESC_COMPLETED BIT(2)
#define XDMA_CTRL_IE_ALIGN_MISMATCH BIT(3)
#define XDMA_CTRL_IE_MAGIC_STOPPED BIT(4)
#define XDMA_CTRL_IE_READ_ERROR    GENMASK(13, 9)
#define XDMA_CTRL_IE_DESC_ERROR    GENMASK(23, 19)
#define XDMA_CTRL_EVENT_MASK       (XDMA_CTRL_IE_DESC_STOPPED | \
				    XDMA_CTRL_IE_DESC_COMPLETED | \
				    XDMA_CTRL_IE_ALIGN_MISMATCH | \
				    XDMA_CTRL_IE_MAGIC_STOPPED | \
				    XDMA_CTRL_IE_READ_ERROR | \
				    XDMA_CTRL_IE_DESC_ERROR)
#define XDMA_CTRL_START            (XDMA_CTRL_RUN_STOP | XDMA_CTRL_EVENT_MASK)

#define XDMA_STAT_BUSY             BIT(0)
#define XDMA_STAT_DESC_STOPPED     BIT(1)
#define XDMA_STAT_DESC_COMPLETED   BIT(2)
#define XDMA_STAT_ALIGN_MISMATCH   BIT(3)
#define XDMA_STAT_MAGIC_STOPPED    BIT(4)
#define XDMA_STAT_INVALID_LEN      BIT(5)
#define XDMA_STAT_H2C_READ_ERROR   GENMASK(13, 9)
#define XDMA_STAT_H2C_WRITE_ERROR  GENMASK(15, 14)
#define XDMA_STAT_DESC_ERROR       GENMASK(23, 19)
#define XDMA_STAT_ERROR_MASK       (XDMA_STAT_ALIGN_MISMATCH | \
				    XDMA_STAT_MAGIC_STOPPED | \
				    XDMA_STAT_INVALID_LEN | \
				    XDMA_STAT_H2C_READ_ERROR | \
				    XDMA_STAT_H2C_WRITE_ERROR | \
				    XDMA_STAT_DESC_ERROR)

#define XDMA_DESC_MAGIC            0xad4b0000u
#define XDMA_DESC_STOP             BIT(0)
#define XDMA_DESC_COMPLETED        BIT(1)
#define XDMA_DESC_EOP              BIT(4)
#define XDMA_ID_MASK               0xffff0000u
#define XDMA_ID_H2C                0x1fc00000u

/* Current zcu106_audio XDMA mapping: BAR1 is the internal DMA engine. */
static unsigned int xdma_bar = 1;
module_param(xdma_bar, uint, 0444);
MODULE_PARM_DESC(xdma_bar, "BAR containing XDMA engine registers");

/* Current zcu106_audio XDMA mapping: BAR0 is the external AXI-Lite master. */
static unsigned int user_bar = 0;
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

static unsigned int bar1_msix_offset;
module_param(bar1_msix_offset, uint, 0444);
MODULE_PARM_DESC(bar1_msix_offset, "Optional BAR1 MSI-X table/PBA offset; current zcu106_audio image has MSI-X disabled, so keep 0");

static bool strict_bar_check = false;
module_param(strict_bar_check, bool, 0444);
MODULE_PARM_DESC(strict_bar_check, "Fail probe if XDMA BAR reads as all ones");

static int rk_xdma_bound_devices;

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

static void xdma_engine_stop(struct rk_xdma_dev *rxd)
{
	/* A read from the same engine flushes PCIe posted MMIO writes. */
	xdma_engine_write(rxd, XDMA_ENGINE_CONTROL, XDMA_CTRL_EVENT_MASK);
	(void)xdma_engine_read(rxd, XDMA_ENGINE_STATUS);
}

static u32 rk_xdma_bar_read32(struct rk_xdma_dev *rxd, unsigned int bar,
			      u32 off)
{
	if (bar >= RK_XDMA_MAX_BARS || !rxd->bar[bar] ||
	    off > rxd->bar_len[bar] || 4 > rxd->bar_len[bar] - off)
		return U32_MAX;
	return ioread32(rxd->bar[bar] + off);
}

static bool rk_xdma_bar_looks_dead(struct rk_xdma_dev *rxd, unsigned int bar)
{
	return rk_xdma_bar_read32(rxd, bar, 0x00) == U32_MAX &&
	       rk_xdma_bar_read32(rxd, bar, XDMA_ENGINE_CONTROL) == U32_MAX &&
	       rk_xdma_bar_read32(rxd, bar, XDMA_ENGINE_STATUS) == U32_MAX &&
	       rk_xdma_bar_read32(rxd, bar, XDMA_COMPLETED_DESC) == U32_MAX;
}

static void rk_xdma_probe_bar_windows(struct rk_xdma_dev *rxd)
{
	unsigned int bar;

	for (bar = 0; bar < RK_XDMA_MAX_BARS; bar++) {
		if (!rxd->bar[bar])
			continue;
		dev_info(&rxd->pdev->dev,
			 "BAR%u probe: [0]=0x%08x ctrl=0x%08x status=0x%08x completed=0x%08x first_desc_lo=0x%08x\n",
			 bar, rk_xdma_bar_read32(rxd, bar, 0x00),
			 rk_xdma_bar_read32(rxd, bar, XDMA_ENGINE_CONTROL),
			 rk_xdma_bar_read32(rxd, bar, XDMA_ENGINE_STATUS),
			 rk_xdma_bar_read32(rxd, bar, XDMA_COMPLETED_DESC),
			 rk_xdma_bar_read32(rxd, bar, XDMA_FIRST_DESC_LO));
	}
}

static void xdma_log_h2c_status(struct rk_xdma_dev *rxd, const char *tag)
{
	u32 control;

	control = xdma_engine_read(rxd, XDMA_ENGINE_CONTROL);
	rxd->last_h2c_status = xdma_engine_read(rxd, XDMA_ENGINE_STATUS);
	rxd->last_h2c_completed = xdma_engine_read(rxd, XDMA_COMPLETED_DESC);
	dev_info(&rxd->pdev->dev,
		 "%s H2C%u control=0x%08x status=0x%08x completed=%u\n",
		 tag, h2c_channel, control, rxd->last_h2c_status,
		 rxd->last_h2c_completed);
}

static void xdma_log_h2c_error(struct rk_xdma_dev *rxd, const char *tag,
			       u32 status)
{
	dev_err(&rxd->pdev->dev,
		"%s H2C%u status=0x%08x busy=%u stopped=%u completed=%u align=%u magic=%u invalid_len=%u read_err=0x%x write_err=0x%x desc_err=0x%x\n",
		tag, h2c_channel, status, !!(status & XDMA_STAT_BUSY),
		!!(status & XDMA_STAT_DESC_STOPPED),
		!!(status & XDMA_STAT_DESC_COMPLETED),
		!!(status & XDMA_STAT_ALIGN_MISMATCH),
		!!(status & XDMA_STAT_MAGIC_STOPPED),
		!!(status & XDMA_STAT_INVALID_LEN),
		(status & XDMA_STAT_H2C_READ_ERROR) >> 9,
		(status & XDMA_STAT_H2C_WRITE_ERROR) >> 14,
		(status & XDMA_STAT_DESC_ERROR) >> 19);
}

static int h2c_submit(struct rk_xdma_dev *rxd, u64 bytes, u64 fpga_addr,
		      u32 timeout_ms)
{
	ktime_t deadline;
	u32 before, control, status;
	u32 desc_lo, desc_hi, desc_adjacent;
	int ret = -ETIMEDOUT;

	if (!rxd->bar[xdma_bar])
		return -ENODEV;
	if (!rxd->buf_virt || !rxd->desc_virt)
		return -ENOMEM;
	if (!bytes || bytes > rxd->buf_size || bytes > U32_MAX)
		return -EINVAL;

	memset(rxd->desc_virt, 0, sizeof(*rxd->desc_virt));
	rxd->desc_virt->control = cpu_to_le32(XDMA_DESC_MAGIC | XDMA_DESC_STOP |
					      XDMA_DESC_COMPLETED | XDMA_DESC_EOP);
	rxd->desc_virt->bytes = cpu_to_le32((u32)bytes);
	rxd->desc_virt->src_addr_lo = cpu_to_le32(lower_32_bits(rxd->buf_dma));
	rxd->desc_virt->src_addr_hi = cpu_to_le32(upper_32_bits(rxd->buf_dma));
	rxd->desc_virt->dst_addr_lo = cpu_to_le32(lower_32_bits(fpga_addr));
	rxd->desc_virt->dst_addr_hi = cpu_to_le32(upper_32_bits(fpga_addr));

	dma_wmb();

	/* Stop a previous timed-out run and clear only stale status events. */
	xdma_engine_stop(rxd);
	udelay(10);
	xdma_log_h2c_status(rxd, "before-submit");
	if (rxd->last_h2c_status)
		(void)xdma_engine_read(rxd, XDMA_ENGINE_STATUS_RC);

	before = xdma_engine_read(rxd, XDMA_COMPLETED_DESC);
	xdma_engine_write(rxd, XDMA_FIRST_DESC_LO, lower_32_bits(rxd->desc_dma));
	xdma_engine_write(rxd, XDMA_FIRST_DESC_HI, upper_32_bits(rxd->desc_dma));
	xdma_engine_write(rxd, XDMA_FIRST_DESC_ADJACENT, 0);

	/* Preserve ordering between descriptor pointer writes and RUN_STOP. */
	wmb();
	(void)xdma_engine_read(rxd, XDMA_ENGINE_STATUS);
	desc_lo = xdma_engine_read(rxd, XDMA_FIRST_DESC_LO);
	desc_hi = xdma_engine_read(rxd, XDMA_FIRST_DESC_HI);
	desc_adjacent = xdma_engine_read(rxd, XDMA_FIRST_DESC_ADJACENT);
	dev_info(&rxd->pdev->dev,
		 "H2C%u descriptor dma=%pad ctrl=0x%08x bytes=%u src=0x%08x%08x dst=0x%08x%08x first_desc=0x%08x%08x adjacent=%u alignments=0x%08x\n",
		 h2c_channel, &rxd->desc_dma,
		 le32_to_cpu(rxd->desc_virt->control),
		 le32_to_cpu(rxd->desc_virt->bytes),
		 le32_to_cpu(rxd->desc_virt->src_addr_hi),
		 le32_to_cpu(rxd->desc_virt->src_addr_lo),
		 le32_to_cpu(rxd->desc_virt->dst_addr_hi),
		 le32_to_cpu(rxd->desc_virt->dst_addr_lo),
		 desc_hi, desc_lo, desc_adjacent,
		 xdma_engine_read(rxd, XDMA_ENGINE_ALIGNMENTS));

	xdma_engine_write(rxd, XDMA_ENGINE_CONTROL, XDMA_CTRL_START);
	control = xdma_engine_read(rxd, XDMA_ENGINE_CONTROL);
	status = xdma_engine_read(rxd, XDMA_ENGINE_STATUS);
	rxd->last_h2c_completed = xdma_engine_read(rxd, XDMA_COMPLETED_DESC);
	dev_info(&rxd->pdev->dev,
		 "start-submit H2C%u control_write=0x%08x control_read=0x%08x status=0x%08x completed=%u before=%u\n",
		 h2c_channel, (u32)XDMA_CTRL_START, control, status,
		 rxd->last_h2c_completed, before);
	if (!(control & XDMA_CTRL_RUN_STOP)) {
		dev_err(&rxd->pdev->dev,
			"H2C%u RUN_STOP did not latch; control=0x%08x\n",
			h2c_channel, control);
		return -EIO;
	}

	if (!timeout_ms)
		timeout_ms = default_timeout_ms;

	deadline = ktime_add_ms(ktime_get(), timeout_ms);
	do {
		rxd->last_h2c_status = xdma_engine_read(rxd, XDMA_ENGINE_STATUS);
		rxd->last_h2c_completed = xdma_engine_read(rxd, XDMA_COMPLETED_DESC);
		if (rxd->last_h2c_completed != before) {
			ret = 0;
			break;
		}
		if (rxd->last_h2c_status & XDMA_STAT_ERROR_MASK) {
			ret = -EIO;
			break;
		}
		usleep_range(50, 100);
	} while (ktime_before(ktime_get(), deadline));

	control = xdma_engine_read(rxd, XDMA_ENGINE_CONTROL);
	desc_lo = xdma_engine_read(rxd, XDMA_FIRST_DESC_LO);
	desc_hi = xdma_engine_read(rxd, XDMA_FIRST_DESC_HI);
	desc_adjacent = xdma_engine_read(rxd, XDMA_FIRST_DESC_ADJACENT);
	if (!ret) {
		dev_info(&rxd->pdev->dev,
			 "H2C complete status=0x%08x completed=%u before=%u control=0x%08x\n",
			 rxd->last_h2c_status, rxd->last_h2c_completed,
			 before, control);
	} else {
		xdma_log_h2c_error(rxd,
			ret == -ETIMEDOUT ? "H2C-timeout" : "H2C-error",
			rxd->last_h2c_status);
		dev_err(&rxd->pdev->dev,
			"H2C final completed=%u before=%u control=0x%08x first_desc=0x%08x%08x adjacent=%u\n",
			rxd->last_h2c_completed, before, control, desc_hi,
			desc_lo, desc_adjacent);
	}

	xdma_engine_stop(rxd);
	return ret;
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
		u64 real_off;

		if (copy_from_user(&io, (void __user *)arg, sizeof(io))) {
			ret = -EFAULT;
			break;
		}
		real_off = io.offset;
		if (io.bar == 1 && bar1_msix_offset)
			real_off += bar1_msix_offset;
		addr = bar_ptr(rxd, io.bar, real_off, io.width);
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
		u64 real_off;

		if (copy_from_user(&io, (void __user *)arg, sizeof(io))) {
			ret = -EFAULT;
			break;
		}
		real_off = io.offset;
		if (io.bar == 1 && bar1_msix_offset)
			real_off += bar1_msix_offset;
		addr = bar_ptr(rxd, io.bar, real_off, io.width);
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
	u16 pci_command;
	u32 engine_id;
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
	pci_read_config_word(pdev, PCI_COMMAND, &pci_command);
	dev_info(&pdev->dev,
		 "PCI command after pci_set_master=0x%04x memory=%u bus_master=%u\n",
		 pci_command, !!(pci_command & PCI_COMMAND_MEMORY),
		 !!(pci_command & PCI_COMMAND_MASTER));

	bars = pci_select_bars(pdev, IORESOURCE_MEM);
	dev_info(&pdev->dev, "memory BAR mask=0x%x xdma_bar=%u user_bar=%u h2c_channel=%u bar1_msix_offset=0x%x\n",
		 bars, xdma_bar, user_bar, h2c_channel, bar1_msix_offset);
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

	rk_xdma_probe_bar_windows(rxd);
	engine_id = xdma_engine_read(rxd, 0x00);
	dev_info(&pdev->dev,
		 "selected XDMA H2C%u engine id=0x%08x alignments=0x%08x\n",
		 h2c_channel, engine_id,
		 xdma_engine_read(rxd, XDMA_ENGINE_ALIGNMENTS));
	if ((engine_id & XDMA_ID_MASK) != XDMA_ID_H2C) {
		dev_err(&pdev->dev,
			"BAR%u offset 0x%x is not an H2C engine: id=0x%08x expected upper16=0x1fc0\n",
			xdma_bar, XDMA_H2C_BASE(h2c_channel), engine_id);
		if (strict_bar_check)
			return -ENODEV;
	}
	if (rk_xdma_bar_looks_dead(rxd, xdma_bar)) {
		dev_err(&pdev->dev,
			"xdma_bar=%u reads all 0xffffffff; PCIe config/link is up but BAR MMIO is not responding. Check FPGA bitstream, XDMA BAR config, AXI clock/reset, and PERST timing.\n",
			xdma_bar);
		if (strict_bar_check)
			return -ENODEV;
	}
	if (user_bar < RK_XDMA_MAX_BARS && rxd->bar[user_bar] &&
	    user_bar != xdma_bar && rk_xdma_bar_looks_dead(rxd, user_bar))
		dev_warn(&pdev->dev,
			 "user_bar=%u also reads all 0xffffffff; control BRAM BAR is not reachable from RK PCIe MMIO.\n",
			 user_bar);

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
	rk_xdma_bound_devices++;
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
	rk_xdma_bound_devices--;
}

static const struct pci_device_id rk_xdma_ids[] = {
	{ PCI_DEVICE(0x10ee, 0x9012) },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, rk_xdma_ids);

static struct pci_driver rk_xdma_pci_driver = {
	.name = DRV_NAME,
	.id_table = rk_xdma_ids,
	.probe = rk_xdma_probe,
	.remove = rk_xdma_remove,
};

static int __init rk_xdma_init(void)
{
	int ret;

	rk_xdma_bound_devices = 0;
	pr_info(DRV_NAME ": loading version %s strict_bar_check=%d\n",
		DRV_VERSION, strict_bar_check);
	ret = pci_register_driver(&rk_xdma_pci_driver);
	if (ret)
		return ret;

	if (strict_bar_check && !rk_xdma_bound_devices) {
		pci_unregister_driver(&rk_xdma_pci_driver);
		pr_err(DRV_NAME ": strict_bar_check requested, but no XDMA endpoint passed BAR probing\n");
		return -ENODEV;
	}
	if (!rk_xdma_bound_devices)
		pr_warn(DRV_NAME ": no matching 10ee:9012 endpoint found; check lspci and PCIe link/reset\n");

	return 0;
}

static void __exit rk_xdma_exit(void)
{
	pci_unregister_driver(&rk_xdma_pci_driver);
}

module_init(rk_xdma_init);
module_exit(rk_xdma_exit);

MODULE_AUTHOR("Codex");
MODULE_DESCRIPTION("RK3568 ARM PCIe host driver for Xilinx XDMA FPGA endpoint");
MODULE_VERSION(DRV_VERSION);
MODULE_LICENSE("GPL");
