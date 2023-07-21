#!/bin/bash

# Step 1: Create a mount point directory if it doesn't exist
MOUNT_POINT="/mnt/usb_drive"
if [ ! -d "$MOUNT_POINT" ]; then
  sudo mkdir -p "$MOUNT_POINT"
fi

# Step 2: Plug in the USB drive to your Raspberry Pi.

# Step 3: Find the device name assigned to your USB drive
DEVICE_NAME=$(lsblk | grep "58.6G" | awk '{print $1}')
if [ -z "$DEVICE_NAME" ]; then
  echo "USB drive not found. Please make sure it is connected."
  exit 1
fi

# Step 4: Mount the USB drive
sudo mount -o umask=000 "$DEVICE_NAME"1 "$MOUNT_POINT"

# Step 5: Verify that the USB drive is mounted successfully
echo "USB drive mounted successfully."
df -h | grep "$MOUNT_POINT"

# You can now access the USB drive contents by navigating to the mount point directory (/mnt/usb_drive).

# Note: To change permissions on the drive to allow anyone to read and write to the drive, you can uncomment the following line:
sudo chmod a+rwx "$MOUNT_POINT"

# To unmount the USB drive before physically removing it, use the following command:
# sudo umount "$MOUNT_POINT"
