#!/bin/bash
# Step 1: Create a mount point directory if it doesn't exist
MOUNT_POINT="/mnt/usb_drive"
if [ ! -d "$MOUNT_POINT" ]; then
  sudo mkdir -p "$MOUNT_POINT"
fi
# Step 2: Plug in the USB drive to your Raspberry Pi.
# Step 3: Check if the USB drive is already mounted
if mountpoint -q "$MOUNT_POINT"; then
  echo "USB drive is already mounted at $MOUNT_POINT."
  exit 1
fi
# Step 4: Find the device name assigned to your USB drive
DEVICE_NAME=$(lsblk -o NAME,SIZE -nr | awk '$2 == "58.6G" {print $1}')
if [ -z "$DEVICE_NAME" ]; then
  echo "USB drive not found. Please make sure it is connected."
  exit 1
fi
# Step 5: Mount the USB drive
sudo mount -o umask=000 "/dev/$DEVICE_NAME" "$MOUNT_POINT"
if [ $? -ne 0 ]; then
  echo "Failed to mount the USB drive."
  exit 1
fi
# Step 6: Verify that the USB drive is mounted successfully
echo "USB drive mounted successfully."
df -h | grep "$MOUNT_POINT"
# You can now access the USB drive contents by navigating to the mount point directory (/mnt/usb_drive).
# Note: To change permissions on the drive to allow anyone to read and write to the drive, you can uncomment the following line:
sudo chmod a+rwx "$MOUNT_POINT"
# To unmount the USB drive before physically removing it, use the following command:
# sudo umount "$MOUNT_POINT"