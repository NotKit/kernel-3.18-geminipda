SDCARDFS_VERSION="0.1"

ccflags-y += -DSDCARDFS_VERSION=\"$(SDCARDFS_VERSION)\"

obj-$(CONFIG_SDCARD_FS) += sdcardfs.o

sdcardfs-y := dentry.o file.o inode.o main.o super.o lookup.o mmap.o packagelist.o derived_perm.o worker.o
