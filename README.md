# maverick-puppet
UAV Companion Computer automation

This is a proof of concept project to automate the installation, configuration, setup and management of UAV companion computers using the Configuration Management system Puppet.

Instead of creating and distributing large complete OS images, this concept allows agile, collaborative development of companion computers through more familiar coding and git workflow.

Puppet is a declarative Configuration Management system that is used to completely configure and manage Operating Systems through code (manifests, in puppet speak).  Instead of defining iterative steps to reach an expected state familiar in traditional coding, instead puppet manifests are written to declare an end state and leaves the puppet system itself to work out how to reach that end state.

Puppet is implemented largely through modules, and there is an extensive public module library (puppet forge) to draw on.  Modules are combined like ingredients to create an overall recipe - a fully configured computer.

Raspberry Pi Installation
------------
Firstly download and install a fresh copy of raspbian (https://www.raspberrypi.org/documentation/installation/installing-images/README.md)
Then login through serial console or ssh (ssh pi@raspberrypi.home)
```
sudo apt-get update
sudo apt-get install git
git clone https://github.com/fnoop/maverick-puppet.git
cd maverick-puppet
sudo ./runme.sh --env=production --confirm
sudo reboot
```
The first run must take place as above, production mode then a reboot.  This is to ensure the base system is setup correctly and the root filesystem is expanded, so there is space for the dev environment.  After the first reboot, the dev environment can be installed if required:
```
sudo ./runme.sh --env=dev --confirm
```
