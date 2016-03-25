# Maverick
UAV Companion Computer automation

This is a proof of concept project to automate the installation, configuration, setup and management of UAV companion computers using Configuration Management.  The name Maverick reflects the initial main goal of interfacing Companion Computers to Flight Controllers through the Mavlink protocol.

Instead of creating, distributing and maintaining large complete OS images and package repositories for each different type of computer and OS, this concept allows agile, collaborative development of companion computers through more familiar coding and git workflow, and is OS and hardware agnostic.  The same environment is created on any supported computer, on any supported OS, so the underlying platform becomes irrelevent.  It is hoped that this will provide the way forward for the evolution of Companion Computers and APM, potentially the next Big Thing(TM) for the APM (aka Ardupilot) project.

The idea is that by downloading the Maverick code and running a single command, anyone can setup a fully functioning Companion Computer (CC), no other technical expertise is needed.  Advanced users can customise the system by specifying parameters to existing classes/modules or by including new ones.  There is also a full development environment available - again through a single command - including a full-featured IDE and APM, Dronekit, ROS and OpenCV code trees, which will make it more convenient and consistent for existing developers and also make it far easier for new developers to start coding. 

This concept uses Puppet, which seems to be the most popular Configuration Management system (others include chef).  Puppet is a declarative Configuration Management system that is used to completely configure and manage Operating Systems and other software through code (manifests, in puppet speak).  Instead of defining iterative steps to reach an expected state familiar in traditional coding, puppet manifests are written to declare an end state and leaves the puppet system itself to work out how to reach that end state.  Because the system is declarative, you can then run puppet as many times as you want and it will reach the same end state, but it only does what needs to be done.  In other words if nothing has changed, it will do and change nothing.  If a bit of puppet code has changed it will only change that bit of the system that the new code relates to, so updates are fast and safe.  Puppet is implemented largely through modules, and there is an extensive public module library (puppet forge) to draw on.  Modules are combined like ingredients to create an overall recipe - a fully configured, running computer.

Maverick is designed to be cross platform.  Initially it is being developed and tested on the following platforms:
 - Beaglebone Black (Debian and Ubuntu)
 - Raspberry Pi (2 Model B, A+, Raspbian and Ubuntu)
 - ODROID (XU3/XU4, Ubuntu)
https://github.com/fnoop/maverick/wiki/Supported-Systems

But it is eventually intended to support all of the following:
 - Navio (Raspberry based) Flight Controllers
 - Erle (Raspberry based) Flight Controllers
 - Linux Desktop/Laptop GCS
 - Mac OSX Desktop/Laptop GCS
 - Nvidia TX1 Jetson
 - Snickerdoodle FPGA
 - And more!

It will detect the base hardware platform and apply default settings as necessary.  Further base and peripheral config is available by specifying config parameters and including new modules.

Initial Installation
--------------------
For debian based distributions:
```
sudo apt-get update && sudo apt-get install git
git clone https://github.com/fnoop/maverick.git; cd maverick
sudo ./maverick --env=bootstrap --confirm
sudo reboot
```
The first run must take place as above, bootstrap mode then a reboot.  This is to ensure the base system is setup correctly and the root filesystem is expanded, so there is space for the chosen working environment.

After the first reboot, the required environment can then be configured:
```
maverick --env=dev --confirm
```
Or production environment (for safer flying):
```
maverick --env=production --confirm
```
To update to the latest Maverick:
```
maverick --self-update
```
Then call maverick with just --confirm flag to update the current environment:
```
maverick --confirm
```

