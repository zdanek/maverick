# maverick-puppet
UAV Companion Computer automation

This is a proof of concept project to automate the installation, configuration, setup and management of UAV companion computers using the Configuration Management system Puppet.

Instead of creating and distributing large complete OS images, this concept allows agile, collaborative development of companion computers through more familiar coding and git workflow.

Puppet is a declarative Configuration Management system that is used to completely configure and manage Operating Systems through code (manifests, in puppet speak).  Instead of defining iterative steps to reach an expected state familiar in traditional coding, instead puppet manifests are written to declare an end state and leaves the puppet system itself to work out how to reach that end state.

Puppet is implemented largely through modules, and there is an extensive public module library (puppet forge) to draw on.  Modules are combined like ingredients to create an overall recipe - a fully configured computer.

Installation
------------
```
sudo apt-get update
sudo apt-get install git
git clone https://github.com/fnoop/maverick-puppet.git
cd maverick-puppet
sudo ./runme.sh
```
eg. ``` sudo ./runme.sh --env=production --confirm```
