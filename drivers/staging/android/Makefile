obj-$(CONFIG_ANDROID_BINDER_IPC)	+= binder.o
obj-$(CONFIG_ASHMEM)			+= ashmem.o
obj-$(CONFIG_ANDROID_LOGGER)		+= logger.o
obj-$(CONFIG_ANDROID_PERSISTENT_RAM)	+= persistent_ram.o
obj-$(CONFIG_ANDROID_RAM_CONSOLE)	+= ram_console.o
obj-$(CONFIG_ANDROID_TIMED_OUTPUT)	+= timed_output.o
obj-$(CONFIG_ANDROID_TIMED_GPIO)	+= timed_gpio.o
obj-$(CONFIG_ANDROID_LOW_MEMORY_KILLER)	+= lowmemorykiller.o
obj-$(CONFIG_ANDROID_SWITCH)		+= switch/
obj-$(CONFIG_ANDROID_INTF_ALARM_DEV)	+= alarm-dev.o
obj-$(CONFIG_PERSISTENT_TRACER)		+= trace_persistent.o
# Bright Lee, 20111114, mmap modem memory for log master {
obj-$(CONFIG_ANDROID_MODEM_MEMORY_LOGGER) += logger_modem.o
# } Bright Lee, 20111114
# Terry add for kernel event trigger
obj-$(CONFIG_ANDROID_KERNEL_EVENT_DRIVER) += kernel_event.o
CFLAGS_REMOVE_trace_persistent.o = -pg
