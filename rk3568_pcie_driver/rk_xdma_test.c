// SPDX-License-Identifier: MIT
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "rk_xdma_ioctl.h"

static int bar_write32(int fd, uint32_t bar, uint64_t off, uint32_t val)
{
	struct rk_xdma_bar_io io = {
		.bar = bar,
		.width = 4,
		.offset = off,
		.value = val,
	};

	return ioctl(fd, RK_XDMA_IOC_BAR_WRITE, &io);
}

static int bar_read32(int fd, uint32_t bar, uint64_t off, uint32_t *val)
{
	struct rk_xdma_bar_io io = {
		.bar = bar,
		.width = 4,
		.offset = off,
	};
	int ret = ioctl(fd, RK_XDMA_IOC_BAR_READ, &io);

	if (!ret)
		*val = io.value;
	return ret;
}

static void usage(const char *prog)
{
	fprintf(stderr,
		"Usage:\n"
		"  %s ctrl <bar> <offset> <value>\n"
		"  %s ctrl-read <bar> <offset>\n"
		"  %s bar-scan\n"
		"  %s h2c <file> <fpga_addr> [timeout_ms]\n"
		"  %s status\n"
		"\n"
		"Current zcu106_audio BD map:\n"
		"  user_bar default is BAR1; offset 0 -> FPGA AXI 0x44a00000 BRAM (driver auto-adds MSI-X offset)\n"
		"  H2C DMA DDR destination starts at FPGA AXI 0x00000000 (2 GB DDR4 space)\n"
		"  Module param: bar1_msix_offset=0x2000 (BAR1 MSI-X table+PBA overhead)\n",
		prog, prog, prog, prog, prog);
}

static int print_status(int fd)
{
	struct rk_xdma_status st;

	if (ioctl(fd, RK_XDMA_IOC_STATUS, &st)) {
		perror("status");
		return 1;
	}
	printf("H2C status=0x%08x completed_desc=%u\n",
	       st.h2c_status, st.h2c_completed_desc);
	if (st.h2c_status == 0xffffffff &&
	    st.h2c_completed_desc == 0xffffffff)
		fprintf(stderr,
			"warning: XDMA register BAR reads all 1s; PCIe enumerated, but BAR MMIO is not responding\n");
	return 0;
}

static int bar_scan(int fd)
{
	static const uint64_t offsets[] = {
		0x00, 0x04, 0x40, 0x48, 0x80, 0x84, 0x88,
	};
	static const uint64_t bar1_extra[] = {
		0x2000, 0x2004,
	};
	unsigned int bar, i;

	for (bar = 0; bar < 6; bar++) {
		int ok = 0;
		int all_ones = 1;

		printf("BAR%u:\n", bar);
		for (i = 0; i < sizeof(offsets) / sizeof(offsets[0]); i++) {
			uint32_t value = 0;

			if (bar_read32(fd, bar, offsets[i], &value)) {
				printf("  [0x%04llx] ERR %s\n",
				       (unsigned long long)offsets[i],
				       strerror(errno));
				all_ones = 0;
				continue;
			}
			ok = 1;
			if (value != 0xffffffff)
				all_ones = 0;
			printf("  [0x%04llx] 0x%08x\n",
			       (unsigned long long)offsets[i], value);
		}
		if (ok && all_ones)
			printf("  -> mapped, but every sampled register is 0xffffffff; BAR MMIO is not responding\n");
		else if (!ok)
			printf("  -> not mapped by driver or invalid BAR\n");

		if (bar == 1) {
			printf("  BAR1 MSI-X adjusted window (offset +0x2000):\n");
			for (i = 0; i < sizeof(bar1_extra) / sizeof(bar1_extra[0]); i++) {
				uint32_t value = 0;
				if (bar_read32(fd, bar, bar1_extra[i], &value)) {
					printf("    [0x%04llx] ERR %s\n",
					       (unsigned long long)bar1_extra[i],
					       strerror(errno));
				} else {
					printf("    [0x%04llx] 0x%08x\n",
					       (unsigned long long)bar1_extra[i], value);
				}
			}
		}
	}

	return 0;
}

int main(int argc, char **argv)
{
	const char *dev = "/dev/rk_xdma0";
	int fd;

	if (argc < 2) {
		usage(argv[0]);
		return 1;
	}

	fd = open(dev, O_RDWR);
	if (fd < 0) {
		perror(dev);
		return 1;
	}

	if (!strcmp(argv[1], "ctrl")) {
		uint32_t bar, value, readback;
		uint64_t off;

		if (argc != 5) {
			usage(argv[0]);
			return 1;
		}
		bar = strtoul(argv[2], NULL, 0);
		off = strtoull(argv[3], NULL, 0);
		value = strtoul(argv[4], NULL, 0);

		if (bar_write32(fd, bar, off, value)) {
			perror("BAR write");
			return 1;
		}
		if (bar_read32(fd, bar, off, &readback)) {
			perror("BAR read");
			return 1;
		}
		printf("BAR%u[0x%llx] = 0x%08x\n",
		       bar, (unsigned long long)off, readback);
	} else if (!strcmp(argv[1], "ctrl-read")) {
		uint32_t bar, value;
		uint64_t off;

		if (argc != 4) {
			usage(argv[0]);
			return 1;
		}
		bar = strtoul(argv[2], NULL, 0);
		off = strtoull(argv[3], NULL, 0);
		if (bar_read32(fd, bar, off, &value)) {
			perror("BAR read");
			return 1;
		}
		printf("BAR%u[0x%llx] = 0x%08x\n",
		       bar, (unsigned long long)off, value);
	} else if (!strcmp(argv[1], "bar-scan")) {
		if (argc != 2) {
			usage(argv[0]);
			return 1;
		}
		return bar_scan(fd);
	} else if (!strcmp(argv[1], "h2c")) {
		struct rk_xdma_dma_alloc alloc;
		struct rk_xdma_h2c h2c;
		char *buf;
		FILE *fp;
		long fsize;
		size_t got;

		if (argc < 4 || argc > 5) {
			usage(argv[0]);
			return 1;
		}

		fp = fopen(argv[2], "rb");
		if (!fp) {
			perror(argv[2]);
			return 1;
		}
		if (fseek(fp, 0, SEEK_END) || (fsize = ftell(fp)) <= 0 ||
		    fseek(fp, 0, SEEK_SET)) {
			perror("file size");
			return 1;
		}

		buf = malloc(fsize);
		if (!buf) {
			perror("malloc");
			return 1;
		}
		got = fread(buf, 1, fsize, fp);
		fclose(fp);
		if (got != (size_t)fsize) {
			fprintf(stderr, "short read\n");
			return 1;
		}

		alloc.size = fsize;
		if (ioctl(fd, RK_XDMA_IOC_DMA_ALLOC, &alloc)) {
			perror("DMA alloc");
			return 1;
		}
		if (write(fd, buf, fsize) != fsize) {
			perror("write DMA buffer");
			return 1;
		}

		memset(&h2c, 0, sizeof(h2c));
		h2c.bytes = fsize;
		h2c.fpga_addr = strtoull(argv[3], NULL, 0);
		h2c.timeout_ms = argc == 5 ? strtoul(argv[4], NULL, 0) : 1000;
		if (ioctl(fd, RK_XDMA_IOC_H2C_SUBMIT, &h2c)) {
			perror("H2C submit");
			print_status(fd);
			return 1;
		}
		printf("H2C sent %ld bytes to FPGA AXI 0x%llx\n",
		       fsize, (unsigned long long)h2c.fpga_addr);
		free(buf);
		return print_status(fd);
	} else if (!strcmp(argv[1], "status")) {
		return print_status(fd);
	} else {
		usage(argv[0]);
		return 1;
	}

	close(fd);
	return 0;
}
