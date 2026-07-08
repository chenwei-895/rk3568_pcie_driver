KDIR ?= /root/rk3568_linux_sdk/kernel
ARCH ?= arm64
CROSS_COMPILE ?= /root/rk3568_linux_sdk/prebuilts/gcc/linux-x86/aarch64/gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-

obj-m += rk_xdma.o

.PHONY: all modules clean app

all: modules app

modules:
	$(MAKE) -C $(KDIR) M=$(CURDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules

app:
	$(CROSS_COMPILE)gcc -Wall -Wextra -O2 -o rk_xdma_test rk_xdma_test.c

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean
	$(RM) rk_xdma_test
