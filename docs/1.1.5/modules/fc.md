# Flight Controller Module
The *fc* (Flight Controller) module is very similar to the [Dev module](/modules/dev).  It provides the same architecture of Mavlink proxy, ROS master and MAVROS that the dev module provides, except instead of the SITL environment it connects to the flight controller hardware, usually through serial or USB connection.  

### FC Mavlink Proxy
As explained more fully in the [mavlink module](/modules/mavlink), a Mavlink proxy is used to provide multiple connection points to a Mavlink source, in this case the flight controller.  This is particularly important for the flight controller environment as the mavlink connection is usually serial and cannot otherwise be used by more than connection at once, which is very limiting.  If a proxy is not wanted, it can be disabled by setting a localconf parameter:  
`"maverick_fc::mavlink_active": false`  
The mavlink source, ie. which serial/USB port on the companion computer takes the mavlink data from the flight controller, can be specified by setting localconf parameter:  
`"maverick_fc::mavlink_input": "/dev/ttyAMA0"`  
The baud rate for serial connections can be changed with localconf parameter:  
`"maverick_fc::mavlink_baud": 115200`  
Sensible defaults are set per platform (/dev/ttyS1 for Joule, /dev/ttyAMA0 for Raspberry).  

The proxy type can be changed by setting a localconf parameter:  
`"maverick_fc::mavlink_proxy": "mavproxy"`  
Current supported proxies are: *mavlink-router* (the default), *mavproxy* and *cmavnode*.  The configuration, system services and firewall rules for each proxy are automatically generated and controlled by Maverick.  
Certain values of the proxy configuration can be set (although it is recommended to keep them as default).  
The default starting tcp port that can be used for client connections is set by localconf parameter:  
`"maverick_fc::mavlink_startingtcp": 5770`  
Some proxies can only provide a single tcp endpoint (cmavnode, mavlink-router).  Mavproxy however can provide multiple tcp endpoints and Maverick provides 3 by default, starting at $mavlink_startingtcp, so 5770, 5771 and 5772.  
There are two types of udp endpoints that mavlink proxies can provide - *udp* (udpout) and *udpin*.  Maverick provides three of each udp endpoint type, starting at $mavlink_startingudp value (14570 by default for SITL).  This can be set with a localconf parameter:  
`"maverick_fc::mavlink_startingudp": 14580`  
Three ports are provided for *udp* (udpout) endpoints: 14570, 14571 and 14572.  Three ports are provided for *udpin* endpoints: 14573, 14574, and 14575.  
Note that SITL (Dev) ports have an '8' in common - 57**8**0-57**8**2, 145**8**0-145**8**2,145**8**3-145**8**5.  FC (Flight Controller) ports have an '7' in common - 57**7**0-57**7**2, 145**7**0-145**7**2, 145**7**3-145**7**5.  This is to make the ports cognitively easier to identify.

## ROS
A complete standard ROS environment is provided.  This ROS environment is separate to the SITL ROS environment and can be disabled during build by setting localconf parameter:  
`"maverick_fc::ros_instance": false`  
By default, a full desktop ROS build is provided and if [the desktop](/modules/desktop) is enabled then standard ROS desktop environments like *rviz* can be used.  

### ROS master
A distinct ROS master is started on the standard port: 11311 by default.  This can be changed by setting localconf parameter:  
`"maverick_fc::rosmaster_port": "11311"`  
The ROS master can be turned off by setting localconf parameter:  
`"maverick_fc::rosmaster_active": false`  
Note this is different from disabling the ROS build altogether (the $ros_instance referred to above) - the $rosmaster_active parameter just stops and deactivates the ROS Master so it does not start at boot.  
Note that when using *roslaunch* to start ROS nodes that should communicate with this ROS master, unlike the SITL environment this environment **does not need** to be altered to find the ROS master on the standard port.

### MAVROS
MAVROS is provided to interface ROS to the flight controller through Mavlink.  By default it uses the first tcp endpoint (5770).  This can be set with localconf parameter:  
`"maverick_fc::mavlink_port": 5770`  
The FC MAVROS instance can be disabled by setting localconf parameter:  
`"maverick_fc::mavros_active": false`  
One problem with starting the ROS master and MAVROS node at launch is that roslaunch (used to start ROS nodes including MAVROS) will start it's own master if it does not detect one running.  However, the ROS master takes a while to fully start, especially on slow platforms, so MAVROS will try and launch a conflicting master.  In order to mitigate this issue, there is a configurable startup delay applied to the MAVROS service.  By default it is 10 seconds, but on particularly slow platforms like the Raspberry it is increased by default to 20 seconds.  The higher the delay, the less likely this issue is to occur and the more reliable the platform, but it delays when the MAVROS node can be connected to and used.  This can be altered with a localconf parameter:  
`"maverick_fc::mavros_startup_delay": 60`  

## Python and Dronekit
A Python 'virtual environment' is pre-configured for flight environment.  A separate copy of dronekit python is installed into ~/code/fc, and the system portion of the virtualenv is in ~/.virtualenvs/fc.  To activate the environment, simply type:  
```bash
workon fc
(fc) [dev] [mav@maverick-joule ~/code/sitl]$
```
The (fc) denotes the python fc virtualenv is currently active.  To use the environment in scripts:  
```bash
source /srv/maverick/.virtualenvs/fc/bin/activate
```
The dronekit-python installed into ~/code/fc is sourced from the official dronekit github repo, but can be changed by setting localconf parameter:  
`"maverick_fc::fc_dronekit_source": "http://github.com/myfork/dronekit-python.git"`
