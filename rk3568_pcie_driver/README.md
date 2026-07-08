# RK3568 ARM PCIe host driver for Xilinx XDMA FPGA endpoint

This module is the ARM-side PCIe driver for this topology:

- RK3568: PCIe Root Complex, Linux 5.10 SDK
- FPGA: PCIe Endpoint using Xilinx XDMA IP
- Small control data: ARM accesses FPGA user BAR registers
- Video stream data: ARM submits XDMA H2C transfers into FPGA AXI address space

## Files

- `rk_xdma.c`: Linux PCIe character driver
- `rk_xdma_ioctl.h`: userspace ABI
- `rk_xdma_test.c`: minimal control/DMA test program
- `Makefile`: out-of-tree build against `/root/rk3568_linux_sdk/kernel`

## FPGA/XDMA assumptions

The code assumes the standard Xilinx XDMA AXI-MM register map:

- XDMA H2C engine register BAR: module parameter `xdma_bar`, default `0`
- FPGA user/control register BAR: module parameter `user_bar`, default `2`
- H2C channel: module parameter `h2c_channel`, default `0`

If your XDMA IP maps AXI-Lite user registers to another BAR, load with:

```sh
insmod rk_xdma.ko xdma_bar=0 user_bar=1 h2c_channel=0
```

The PCI ID table matches Xilinx vendor `0x10ee` with any device ID. For a
production system, replace `PCI_ANY_ID` in `rk_xdma_ids` with the exact FPGA
device ID from:

```sh
lspci -nn
```

## RK3568 kernel/device-tree checklist

Make sure the board DTS enables the RK3568 PCIe controller used by the slot or
M.2 connector. The exact node name depends on the SDK board file, but the node
usually needs:

```dts
&pcie2x1 {
	status = "okay";
	reset-gpios = <&gpioX RK_PXn GPIO_ACTIVE_HIGH>;
	vpcie3v3-supply = <&vcc3v3_pcie>;
};
```

Kernel config must include:

```text
CONFIG_PCI=y
CONFIG_PCIE_ROCKCHIP_HOST=y
CONFIG_DMA_SHARED_BUFFER=y
CONFIG_CMA=y
```

Reserve enough contiguous memory for large video buffers. For early testing,
append a CMA size such as this to kernel bootargs:

```text
cma=256M
```

## Build

```sh
cd /root/rk3568_pcie_xdma_arm_driver
make
```

This builds:

- `rk_xdma.ko`
- `rk_xdma_test`

If your SDK uses a different kernel output directory, override `KDIR`:

```sh
make KDIR=/path/to/kernel-build
```

## Load and test

Copy `rk_xdma.ko` and `rk_xdma_test` to the RK3568 root filesystem, then:

```sh
insmod rk_xdma.ko xdma_bar=0 user_bar=2 h2c_channel=0
dmesg | tail -50
ls -l /dev/rk_xdma0
```

Write/read a 32-bit control register in FPGA user BAR:

```sh
./rk_xdma_test ctrl 2 0x00 0x00000001
```

Send a video frame/file to FPGA AXI address `0x00000000`:

```sh
./rk_xdma_test h2c frame.bin 0x00000000 3000
```

## Driver API

The character device is `/dev/rk_xdma0`.

- `RK_XDMA_IOC_BAR_READ`: read 1/2/4 bytes from a mapped BAR
- `RK_XDMA_IOC_BAR_WRITE`: write 1/2/4 bytes to a mapped BAR
- `RK_XDMA_IOC_DMA_ALLOC`: allocate one coherent DMA buffer
- `RK_XDMA_IOC_DMA_FREE`: free the DMA buffer
- `write`: copy userspace data into the coherent DMA buffer
- `mmap`: map the coherent DMA buffer for zero-copy userspace filling
- `RK_XDMA_IOC_H2C_SUBMIT`: submit one H2C descriptor
- `RK_XDMA_IOC_STATUS`: read cached/current H2C status registers

## Next production steps

This first version is a bring-up driver. For continuous video streaming, extend
it with:

- descriptor ring instead of one descriptor
- MSI/MSI-X interrupt completion instead of polling
- multiple video buffers with dequeue/enqueue ioctls
- C2H receive path if the FPGA must send processed frames back to RK3568
- exact PCI device ID matching
