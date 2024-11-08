#!/bin/bash
# Make default camera /dev/video0 point to the desired camera (/dev/video4)
# ref: https://askubuntu.com/questions/396952/how-to-change-the-default-webcam-changing-defaults-in-multimedia-selctor-not-wo


if [ -h /dev/video0 ]; then 
   sudo rm /dev/video0   # not first run: remove our old symlink
elif [ -e /dev/video0 ]; then
   sudo mv /dev/video0 /dev/video0.original  # rename original
fi 
if [ -e /dev/video4 ]; then
   sudo ln -s /dev/video4 /dev/video0   # symlink to video4 if it exists
   echo "Set default camera /dev/video0 --> external camera /dev/video4"
elif [ -e /dev/video0.original ]; then  # symlink to video0.original otherwise
   sudo ln -s /dev/video0.original /dev/video0
   echo "Set default camera /dev/video0 --> integrated camera /dev/video0.original"
else
   echo "No device found"
   ls -l /dev/video*
fi