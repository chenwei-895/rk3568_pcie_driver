#ifndef RK_XDMA_IOCTL_H
#define RK_XDMA_IOCTL_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define RK_XDMA_IOC_MAGIC 'x'

struct rk_xdma_bar_io {
	__u32 bar;
	__u32 width;      /* 1, 2, or 4 bytes */
	__u64 offset;
	__u32 value;
};

struct rk_xdma_dma_alloc {
	__u64 size;
};

struct rk_xdma_h2c {
	__u64 bytes;
	__u64 fpga_addr;  /* XDMA AXI destination address */
	__u32 timeout_ms;
};

struct rk_xdma_status {
	__u32 h2c_status;
	__u32 h2c_completed_desc;
};

#define RK_XDMA_IOC_BAR_READ    _IOWR(RK_XDMA_IOC_MAGIC, 0x01, struct rk_xdma_bar_io)
#define RK_XDMA_IOC_BAR_WRITE   _IOW(RK_XDMA_IOC_MAGIC, 0x02, struct rk_xdma_bar_io)
#define RK_XDMA_IOC_DMA_ALLOC   _IOW(RK_XDMA_IOC_MAGIC, 0x03, struct rk_xdma_dma_alloc)
#define RK_XDMA_IOC_DMA_FREE    _IO(RK_XDMA_IOC_MAGIC, 0x04)
#define RK_XDMA_IOC_H2C_SUBMIT  _IOW(RK_XDMA_IOC_MAGIC, 0x05, struct rk_xdma_h2c)
#define RK_XDMA_IOC_STATUS      _IOR(RK_XDMA_IOC_MAGIC, 0x06, struct rk_xdma_status)

#endif
