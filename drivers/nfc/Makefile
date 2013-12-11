#
# Makefile for nfc devices
#

obj-$(CONFIG_PN544_NFC)		+= pn544.o
# Jen add for factory test scripts
obj-$(CONFIG_PN544_TEST)	+= pn544_test.o
# Jen add 20121217
obj-$(CONFIG_NFC_PN533)		+= pn533.o
obj-$(CONFIG_NFC_WILINK)	+= nfcwilink.o

ccflags-$(CONFIG_NFC_DEBUG) := -DDEBUG
