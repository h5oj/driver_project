
ccflags-y := -Iarch/$(SRCARCH)/xenomai/include -Iinclude/xenomai \
	-Idrivers/xenomai/analogy

obj-$(CONFIG_XENO_DRIVERS_ANALOGY_CB_PCIDAS) += analogy_cb_pcidas.o

analogy_cb_pcidas-y := cb_pcidas.o
