#
# Makefile for the Ingenic JZ4740.
#

# Object file lists.

obj-y += prom.o
obj-y += time.o
obj-y += reset.o
obj-y += setup.o
obj-y += platform.o

CFLAGS_setup.o = -I$(src)/../../../scripts/dtc/libfdt

# board specific support

obj-$(CONFIG_JZ4740_QI_LB60)	+= board-qi_lb60.o

# PM support

obj-$(CONFIG_PM) += pm.o
