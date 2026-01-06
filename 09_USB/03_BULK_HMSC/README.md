- USB for bulk hmsc
- Communication between BBB and USB drive (FAT32 format) in high speed. Apart from FAT32 format partrition, this driver does not support other kinds or can reformat the disk in FAT32 format:
    + sudo mkfs.vfat -F 32 /dev/sdX -> VBR is at sector 0 (No MBR) OR
    + 1. sudo umount ${dev/sda}* 2>/dev/null || true
      2. sudo sfdisk ${DISK} --force <<-__EOF__
         4M,,L,*
         __EOF__ -> /dev/sda1 -> MBR is at sector 0

- In main.h, there is macro 'PRINT_CONTENT' which prints the content within files and 'HIDDEN_FOLDERS' which prints the hidden folders and 'PRINT_USB_INFO' which prints the basic information of usb drive
- In main.c, we can put tests/functions for handling files/folders in requested_work()
- Before loading usb.ko, it's recommended to remove musb_dsps out of kernel or unbind usb modules (maybe by editing device tree in usb@1800)
