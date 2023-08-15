#!/bin/bash

# To unmount the USB drive before physically removing it, use the following command:
MOUNT_POINT="/mnt/usb_drive"
sudo umount "$MOUNT_POINT"
echo "USB drive unmounted from $MOUNT_POINT."
