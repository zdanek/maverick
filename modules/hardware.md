# Hardware Module

## Overview
Almost all of the Maverick code/design is platform agnostic, that is it doesn't care about the underlying hardware.  However, all platforms have differences that need to be worked around, and also there is various setup and configuration that can be automated to make it easier for the end user.  

The hardware module is called for every `maverick configure` run.  It automatically detects the underlying hardware platform and installs any relevant software and performs any relevant setup and configuration.  This includes any peripherals (eg. cameras) that are plugged in and recognized.  

### Intel Joule
The Intel Joule is a tiny, powerful embedded computer.  There are two variants - 550 and 570, both of which are supported by Maverick (although currently the distributed OS Image is too large to fit on the 550.  This will be corrected in a subsequent image).  
The Joule can run various operating systems.  There are two OS variants from Intel based on Yocto, there are two variants from Ubuntu (core and desktop), and it can also run Microsoft Windows.  The Yocto OS is really intended for embedded systems that are quite pre-set in nature and not really intended for interactive use with the end user.  Window is.. Windows.  Ubuntu Core is a somewhat proprietary effort to popularise containers, but it is Ubuntu Desktop that is a relatively traditional Linux OS, and that is what Maverick uses primarily on the Joule.  

Maverick uses the Desktop Joule image found here: https://developer.ubuntu.com/core/get-started/intel-joule, which looks to be the standard Ubuntu 16.04 desktop with a customised kernel to support various Intel platform systems.  Maverick expands the root filesystem on the eMMC (onboard flash storage) if necessary, installs some useful drivers, software and libraries, and removes some unnecessary software to free up disk space.  This software removal can be disabled by setting localconf parameter:  
`"maverick_hardware::joule::remove_more_packages": false`  
The customised kernel includes support for the Gumstix/Intel CSI cameras for the Joule (https://www.gumstix.com/blog/tiny-4k-hd-cameras-intel-joule/).  This support is rather clunky in Ubuntu and floods the v4l2 subsystem with dozens of confusing and unnecessary video devices.  So the support for these cameras is disabled by default by blacklisting the relevent kernel modules.  This can be disabled (thus re-enabling the cameras) by setting localconf parameter:  
`"maverick_hardware::joule::ipu4_blacklist": false`  

### Intel RealSense
Intel RealSense is a very interesting technology from Intel primarily involving depth cameras and sensors that provides intruiging possibilities for robotics, drones and VR.  The cameras are available relatively cheaply and offload a lot of work from the host computer onto the custom camera hardware.  Support is already included in Maverick, including setting up the camera hardware, drivers and supporting software.  If a RealSense camera is attached, Maverick will automatically detect it and install and configure the necessary drivers and software.  The provided Maverick OS image for Joule includes RealSense support.

### Odroid XU3/XU4
The Odroid XU3/XU4 is a small embedded computer the size of a Raspberry Pi, but with considerably more computing power.  It uses an 8-core Samsung ARM CPU.  Maverick uses the experimental Ubuntu 16.04 OS with 4.x kernel image from Odroid, and then installs and configures useful software and libraries.  There is a single localconf parameter that can be set, which determines which power governor to use at boot time:  
`maverick_hardware::odroid::governor_atboot": "performance"`  
It is set to (maximum) *performance* at boot.  Other values are *interactive*, *conservative*, *ondemand*, *powersave*.

### Raspberry Pi
The Raspberry Pi needs no introduction.  It is a very low cost computer and traditional low performance, but recent models (Pi2, Pi3) have introduced quad-core CPUs with reasonable performance.  It also has a well supported GPU and VPU with a cheap CSI camera, so is a cheap and popular method of digital real-time video.  There are low-power, smaller models such as the Model A(+), and more recently the Zero and Zero W.  The only OS for the Raspberry which provides decent hardware support across all models is the foundation supplied Raspbian, a debian based OS.  Maverick takes Raspbian as a base and customizes and builds on it to make a great UAV Companion Computer system.  Maverick has also been lightly tested on Ubuntu for Raspbian as well as Autopilot-oriented OS like those from Erle and Emlid.

The setup for Raspberry Pis depend on which model, and which peripherals are being used.  Maverick sets up sensible defaults for the most likely usage scenario for UAVs, while providing parameters to control configuration if required.

Maverick detects if the root filesystem/partition does not fill the entire disk (SD card), which is common when first booted from a flashed image.  In this case, the system is setup to expand the partition and filesystem on next boot.  This can be disabled by setting localconf parameter:  
`"maverick_hardware::raspberry::expand_root": false`  
The Raspberry GPU shares memory with the main memory pool and this memory split can be altered as desired with a localconf parameter:  
`"maverick_hardware::raspberry::gpumem": 128`  
The CPU can be overclocked if desired by setting localconf parameter:  
`"maverick_hardware::raspberry::overclock": "High"` (valid values are *None*, *High*, *Turbo*). For maximum reliability of the platform it is strongly recommended to leave this as the default *None*.  
Raspbian has a serial console set on the UART exposed on the GPIO pins 8 and 10 by default.  However, this interferes with connecting the Raspberry to a Flight Controller, so is turned off by Maverick.  This can be reversed by setting localconf parameter:  
`"maverick_hardware::raspberry::serialconsole": true`  
USB ports provide a maximum of 500ma current according to the USB protocol - anything more risks instability of the system, particularly as the Raspberry is powered by a USB port itself.  However, with careful hardware setup the Raspberry is capable of providing higher current through the USB ports which is useful for high power peripherals such as Wifi.  In this case, the current limiter can be lifted by setting localconf parameter:  
`"maverick_hardware::raspberry::overpower_usb": true`  

### BeagleBone Black
The BeagleBone Black (BBB) is an interesting embedded computer system about the size of a Raspberry Pi.  It is popular with the hacking community and has been used for several ArduPilot projects.  It has an onboard MMC flash memory but can also be booted from SD card.  Maverick has been developed and tested on BeagleBone Black running the Ubuntu OS.
