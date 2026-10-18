obj-m += axi_dma.o

# Strip GCC 12+ flags if compiling with GCC 11
CFLAGS_REMOVE_axi_dma.o += -ftrivial-auto-var-init=zero \
                           -mstack-protector-guard=tls \
                           -mstack-protector-guard-offset=1320

# Disable stack protector for this module if on GCC 11
CFLAGS_axi_dma.o += -fno-stack-protector

KDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

ARCH ?= arm
CROSS_COMPILE ?= 

ifneq ($(CROSS_COMPILE),)
CC := $(CROSS_COMPILE)gcc
else
CC ?= $(shell which gcc-12 2>/dev/null || echo gcc)
endif

all:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules
	$(CC) -O2 -Wall test_dma.c -lpthread -o test_dma

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	rm -f test_dma
