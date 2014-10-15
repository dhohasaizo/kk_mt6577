#
# Makefile for nfc devices
#

obj-$(CONFIG_PN544_NFC)		+= pn544.o
obj-$(CONFIG_NFC_PN533)		+= pn533.o
obj-$(CONFIG_NFC_WILINK)	+= nfcwilink.o
obj-$(CONFIG_MTK_NFC_INSIDE) += microread.o

ccflags-$(CONFIG_NFC_DEBUG) := -DDEBUG -DEXPORT_SYMTAB
#ccflags-$(CONFIG_MTK_NFC_INSIDE) := -DEXPORT_SYMTAB

microread-objs := open_nfc_main.o
microread-objs += open_nfc_custom.o