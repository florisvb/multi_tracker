To use point grey / FLIR USB3 cameras you can use:

https://github.com/ros-drivers/pointgrey_camera_driver

I can confirm that installing that from source on ubuntu 18.04.2 running melodic does work, after also installing FlyCapture 2.13.3.31 SDK - Linux Ubuntu 18.04 (64-bit) — 10/25/2018.

Tested with a blackfly or flea USB3 (I can't recall how to tell which one).

FlyCapture 2.13.3.31 SDK can be downloaded from: https://www.ptgrey.com/support/downloads
(make a free account first, and follow the prompts to find your product and it will give the appropriate download link). 

It may be necessary to install some other packages first (see: https://www.ptgrey.com/tan/10548):
`user$: sudo apt-get install libraw1394-11 libgtkmm-2.4-1v5 libglademm-2.4-1v5 libgtkglextmm-x11-1.2-dev libgtkglextmm-x11-1.2 libusb-1.0-0`

Then run
`user$: sudo sh install_flycapture.sh`
where install_flycapture.sh is in the FlyCapture 2.13.3.31 SDK download. 

Also note that for USB3 to work properly, you may need to follow these instructions (configuring USB-FS):
https://www.ptgrey.com/KB/10685

To permanently modify the USB-FS memory limit:

In a text editor, open the /etc/default/grub file.
Find
`GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"`
Replace with
`GRUB_CMDLINE_LINUX_DEFAULT="quiet splash usbcore.usbfs_memory_mb=1000"`
Update grub with these settings:
`$ sudo update-grub`
Reboot and test a USB camera.