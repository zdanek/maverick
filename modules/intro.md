# Maverick Modules

Maverick is built up using modules.  Each environment (bootstrap, flight, development) is comprised of a different combination of modules and parameters.  All environments start with the base module:
- [Base](/modules/base)

Depending on the environment and what other classes are included by configuration, these other modules are available:
  - [Hardware](/modules/hardware)
  - [Network](/modules-network)
  - [Security](/modules-security)
  - [Vision](/modules-vision)
  - [Flight Controller](/modules-fc)
  - [ROS](/modules-ros)
  - [Development](/modules-dev)
  - [GCS](/modules-gcs)
