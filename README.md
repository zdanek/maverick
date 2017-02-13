## What is Maverick?
Maverick is a system for creating, installing, configuring, updating and maintaining UAV companion computers, with a high level of automation, consistency and reliability, minimizing risk.  
The name Maverick reflects the initial main goal of interfacing Companion Computers to Flight Controllers through the Mavlink protocol.

### Why Maverick?
UAV Companion Computers suffer from a lack of collaborative development, unlike flight controller development.  Software for companion computers is usually distributed in the form of modified vendor OS images.  These are simple to download and install, but are difficult or clumsy to create and are not upgradable.  If new features are developed, a new image must be released and end-users must reinstall from scratch with the new image.  
Maverick provides easy to install OS images for convenience, but can be entirely installed from a few K of github code.  When new features are developed using Maverick, existing installs can be updated quickly and safely with a single command without any disruption to data or running services.

### What does it do?
Lots!  And lots more planned!
- Configures the computer hardware and peripherals
- Configures network interfaces, addresses, services, wireless setup
- Hardens security, configures firewall, antivirus, intrusion detection, reactive brute-force defences
- Configures flight controller proxy and dronekit environment
- Compiles/installs gstreamer and opencv software, aruco fiducial marker software, FPV intelligent daemon
- Dev environment installs and configures browser IDE (Cloud9), Ardupilot SITL, proxy, dronekit environment
- Installs and configures ROS and Mavros

### How does it work?
At the heart of Maverick is the tried and tested Configuration Management system Puppet.  Puppet is a system that abstracts the configuration of a system into declarative code, and is widely used in complex computing environments to automate computer configuration in a secure, repeatable manner that reduces risk and increases reliability.  Maverick can be used to create and install companion systems, but can also be used to update an existing system.  Maverick can be re-run any number of times and will only make changes where necessary to bring the system into the configured state.

### Hardware support
Maverick is designed to be cross platform and hardware agnostic. Initially it is being developed and tested on the following platforms:
- Beaglebone Black (Debian and Ubuntu)
- Raspberry Pi (3, 2 Model B, A+, Raspbian and Ubuntu)
- ODROID (XU3/XU4, Ubuntu)

Support will be added for more hardware in the future, eg:
- Nvidia TX1
- Intel Edison/Joule
- Up Board/Squared

Maverick will detect the base hardware platform and apply default settings as necessary. Further base and peripheral config is available by specifying config parameters and including new modules.

### OS support
Like hardware support, Maverick is designed to be software agnostic. Currently most of the support is Debian/Ubuntu based, but the underlying Configuration Management system (puppet) that does most of the heavy lifting is completely cross platform and has providers for almost every OS.
Currently it supports:
- Raspbian (Raspberry Pi official debian based OS)
- Odroid Ubuntu (Official ubuntu OS for Odroid XU3/4)
- Beaglebone Black Ubuntu

It has also been lightly tested with:
- Ubuntu for Raspberry
- Odroid XU4 Ubuntu + Experimental 4.9 kernel


## Quick Start
There are two ways to get started with Maverick:  
[Download OS Images](#os-images)  
  *_or_*  
[Run Maverick](#run-Maverick)  

### OS images
This is how most SBC vendors provide software and is the process most people are used to.  
Once the image is downloaded, this method is much faster to get started with than a fresh install running Maverick.  
- Download an .iso, flash to SD card and boot from the <a href="/#/download">download page</a>.

### Run Maverick

!> Warning: This can take a LONG time depending on the environment.  A flight mode install can take around an hour, whereas a full development install on a slow computer like a Raspberry Pi can take over 12 hours.

Maverick can be run from any new or existing OS installation.  The provided OS images are simply a fresh vendor OS with Maverick run and are provided for convenience, as initial Maverick runs can take a long time due to compiling and installing lots of software.  
Once the initial Maverick run is complete, the system should be in exactly the same state as if installed from OS image.  
- First download Maverick and do a bootstrap run and reboot:
```
sudo apt-get update && sudo apt-get -y upgrade
sudo apt-get install -y git && git clone https://github.com/fnoop/maverick.git
cd maverick && sudo ./bin/Maverick --env=bootstrap configure
sudo reboot
```
- Then login as the 'mav' user (default password is 'wingman') and run Maverick with an end-state environment, eg. for flight/production environment:
```
maverick --env=flight configure
```
Or for development environment:
```
maverick --env=dev configure
```

## Quick Update
Once Maverick has been installed, it is very easy to update the system in the future:
- Simply update Maverick itself:
```
maverick self-update
```
- Then rerun Maverick in configure mode:
```
maverick configure
```

## Quick Status
Maverick gives some useful info/status:
- Status of running Maverick related services:
```
maverick status
```
- System info:
```
maverick info
```
- Network info:
```
maverick netinfo
```
