# Hardware Module

## Overview
Almost all of the Maverick code/design is platform agnostic, that is it doesn't care about the underlying hardware.  However, all platforms have differences that need to be worked around, and also there is various setup and configuration that can be automated to make it easier for the end user.  

The hardware module is called for every `maverick configure` run.  It automatically detects the underlying hardware platform and installs any relevant software and performs any relevant setup and configuration.  This includes any peripherals (eg. cameras) that are plugged in and recognized.  
It is also possible to set localconf parameters to force hardware support for each platform/peripheral.  This is useful for platform builders who want to include support for platforms/peripherals without needing to have them all present at build time.

### Raspberry Pi
The Raspberry Pi needs no introduction.  It is a very low cost computer and started off with very low performance, but recent models (Pi3, Pi4) have introduced quad-core CPUs with reasonable performance - the latest Model 4B+ has a fast USB3 bus as well as faster CPU and RAM, more RAM capacity options, built-in WiFi and Bluetooth, dual HDMI and is quite a capable little computer.  It also has a well supported GPU and VPU with a cheap CSI camera, so is a cheap and popular method of digital real-time video.  There are low-power, smaller models such as the Model A(+), and more recently the Zero and Zero W.  The only OS for the Raspberry which provides decent hardware support across all models is the foundation supplied Raspbian, a Debian based OS.  Maverick takes Raspbian as a base and customizes and builds on it to make a great UAV Companion Computer system.  Maverick has also been lightly tested on Ubuntu for Raspbian as well as Autopilot-oriented OS like those from Erle and Emlid.

The setup for Raspberry Pis depend on which model, and which peripherals are being used.  Maverick sets up sensible defaults for the most likely usage scenario for UAVs, while providing parameters to control configuration if required.  To force installation of Raspberry platform support, set localconf parameter:  
`"maverick_hardware::raspberry_install": true`

!> Note: Maverick resets the default 'pi' user password to 'wingman', the same as the mav user.  This is because Raspbian checks if the default password is set and complains if it is.  This is a good idea, as otherwise there may be untold numbers of insecure Raspberries online with default passwords, ready to become zombies.  The default password can be set by specifying an encrypted string in the localconf parameter `"maverick_hardware::raspberry::pi_password"`.

Maverick detects if the root filesystem/partition does not fill the entire disk (SD card), which is common when first booted from a flashed image.  In this case, the system is setup to automatically expand the partition and filesystem on first boot and automatically reboot to activate, which is invisible to the user.  This can be disabled by setting localconf parameter:  
`"maverick_hardware::raspberry::expand_root": false`  
The Raspberry GPU shares memory with the main memory pool and this memory split can be altered as desired with a localconf parameter:  
`"maverick_hardware::raspberry::gpumem": 128`  
The CPU can be overclocked if desired by setting localconf parameter:  
`"maverick_hardware::raspberry::overclock": "High"` (valid values are *None*, *High*, *Turbo*). For maximum reliability of the platform it is strongly recommended to leave this as the default *None*.  
Raspbian has a serial console set on the UART exposed on the GPIO pins 8 and 10 by default.  However, this interferes with connecting the Raspberry to a Flight Controller, so is turned off by Maverick.  This can be reversed by setting localconf parameter:  
`"maverick_hardware::raspberry::serialconsole": true`  
USB ports provide a maximum of 500ma current according to the USB protocol - anything more risks instability of the system, particularly as the Raspberry is powered by a USB port itself.  However, with careful hardware setup the Raspberry is capable of providing higher current through the USB ports which is useful for high power peripherals such as Wifi.  In this case, the current limiter can be lifted by setting localconf parameter:  
`"maverick_hardware::raspberry::overpower_usb": true`  
The default swap size on Raspbian is 100Mb and is implemented using the swapfile /var/swap.  When running memory hungry software such as compilers or running a lot of software which is a common scenario with Maverick, having more swap available is a good idea.  So the default in Maverick is increased to 1024Mb (1Gb).  To alter this, set the localconf parameter:  
`"maverick_hardware::raspberry::swapsize": 512`

On the Raspberry Pi 3 and Pi Zero W, the bluetooth overlay grabs the hardware line for the serial UART (/dev/ttyAMA0) and by default provides an unreliable serial device on /dev/ttyS0.  This serial device /dev/ttyS0 is very sensitive to bus and CPU throttling and becomes unreliable or unuseable when the bus or CPU is throttled (which is normal).  So Maverick disables this overlay and restores the hardware serial UART to /dev/ttyAMA0 where it is reliable.  However, this disables bluetooth.  In order to disable this behaviour and re-enable the bluetooth overlay, set the localconf parameter:  
`"maverick_hardware::raspberry::serialoverride": false`  
Note that this puts the serial UART back on /dev/ttyS0 where it is known to be unreliable.

### Intel Joule
The Intel Joule is a tiny, powerful embedded computer.  There are two variants - 550 and 570, both of which are supported by Maverick.  
The Joule can run various operating systems.  There are two OS variants from Intel based on Yocto, there are two variants from Ubuntu (core and desktop), and it can also run Microsoft Windows.  The Yocto OS is really intended for embedded systems that are quite pre-set in nature and not really intended for interactive use with the end user.  Windows is.. Windows.  Ubuntu Core is a somewhat proprietary effort to popularise containers, but it is Ubuntu Desktop that is a relatively traditional Linux OS, and that is what Maverick uses primarily on the Joule.  To force Joule hardware support to be installed, set localconf parameter:  
`"maverick_hardware::joule_install": true`

Maverick uses the Desktop Joule image found here: https://developer.ubuntu.com/core/get-started/intel-joule, which looks to be the standard Ubuntu 16.04 desktop with a customised kernel to support various Intel platform systems.  Maverick expands the root filesystem on the eMMC (onboard flash storage) if necessary, installs some useful drivers, software and libraries, and removes some unnecessary software to free up disk space.  This software removal can be disabled by setting localconf parameter:  
`"maverick_hardware::joule::remove_more_packages": false`  
The customised kernel includes support for the Gumstix/Intel CSI cameras for the Joule (https://www.gumstix.com/blog/tiny-4k-hd-cameras-intel-joule/).  This support is rather clunky in Ubuntu and floods the v4l2 subsystem with dozens of confusing and unnecessary video devices.  So the support for these cameras is disabled by default by blacklisting the relevent kernel modules.  This can be disabled (thus re-enabling the cameras) by setting localconf parameter:  
`"maverick_hardware::joule::ipu4_blacklist": false`  

### Aaeon Up
The Up boards (http://www.up-board.org/) are Intel CPU based small, inexpensive SBCs (Single Board Computers).  As they are Intel based, they share a lot of the hardware components and therefore OS support as the Intel Joule platform.  All of the current Up boards are supported from a single OS image.

### Odroid XU3/XU4
The Odroid XU3/XU4 is a small embedded computer the size of a Raspberry Pi, but with considerably more computing power.  It uses an 8-core Samsung ARM CPU.  Maverick uses the experimental Ubuntu 16.04 OS with 4.x kernel image from Odroid, and then installs and configures useful software and libraries.  To force install of Odroid support, set localconf parameter:  
`"maverick_hardware::odroid_install": true`  
There is a single localconf parameter that can be set, which determines which power governor to use at boot time:  
`"maverick_hardware::odroid::governor_atboot": "performance"`  
It is set to (maximum) *performance* at boot.  Other values are *interactive*, *conservative*, *ondemand*, *powersave*.
### Odroid oCam
The oCam camera will be automatically detected and support installed.  To force install, set localconf parameter:  
`"maverick_hardware::camera_ocam_install": true`

### BeagleBone Black
The BeagleBone Black (BBB) is an interesting embedded computer system about the size of a Raspberry Pi.  It is popular with the hacking community and has been used for several ArduPilot projects.  It has an onboard MMC flash memory but can also be booted from SD card.  Maverick has been developed and tested on BeagleBone Black running the Ubuntu OS.  Beaglebone hardware will be detected automatically, to force platform support, set localconf parameter:  
`"maverick_hardware::beagle_install": true`  

### Intel RealSense
Intel RealSense is a very interesting technology from Intel primarily involving depth cameras and sensors that provides intruiging possibilities for robotics, drones and VR.  The cameras are available relatively cheaply and offload a lot of work from the host computer onto the custom camera hardware.  Support is already included in Maverick, including setting up the camera hardware, drivers and supporting software.  If a RealSense camera is attached, Maverick will automatically detect it and install and configure the necessary drivers and software.  The provided Maverick OS image for Joule includes RealSense support.  To force install of RealSense support, set localconf parameter:  
`"maverick_hardware::camera_realsense_install": true`

### Seek Thermal cameras
Seek Thermal Compact and CompactPro cameras are supported.  If plugged in they will be automatically detected and support installed and configured at the next `maverick configure` run.  To force install, set localconf parameter:  
`"maverick_hardware::seekthermal_install": true`  
If Maverick does not detect a Seek Thermal camera plugged in, and install is forced as above, it will default to configure a Seek Compact device (id 0010).  To change this to force install support for a Seek Compact Pro (id 0011), set a localconf parameter:  
`"maverick_hardware::peripheral::seekthermal::seek_id": "0011"`  
