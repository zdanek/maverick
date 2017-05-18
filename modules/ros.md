# ROS Module
Like the *mavlink* module, the *ROS* module is called by other modules to create instances of ROS and MAVROS.  However, it has core logic to install the underlying ROS and MAVROS software.  

## ROS Installation
ROS provides extensive binary packages that are frequently updated, matched for an OS version and ROS version, and recommends that these are used instead of compiled versions where possible.  There is a localconf parameter that controls the process of whether to install ROS from source or binary packages:  
`"maverick_ros::installtype": "source"`  
By default, $installtype is set to "", which means autodetect.  The core manifest has a table of ROS-supported OS architectures and versions, and if the platform is supported then it will set ROS for a binary install (eg. Joule and Odroid).  For unsupported platforms, a source install is necessary (eg. Raspberry Pi).
The ROS distribution is currently *Kinetic*.  This can be changed by setting localconf paramter:  
`"maverick_ros::distribution": "jade"`  
There are separate parameters for the installation type: $buildtype and $binarytype.  The default build for source install is 'ros_comm', and can be changed by setting localconf parameter:  
`"maverick_ros::buildtype": "ros_desktop"`  
The default install type for binary packages is 'ros-kinetic-desktop' and can be changed by setting localconf parameter:  
`"maverick_ros::binarytype": "ros-kinetic-desktop-full"`  
ROS (even the binary install) lives in ~/software/ros.  To change this (which is strongly recommended against), set localconf parameter:  
`"maverick_ros::installdir": "/srv/maverick/software/altros"`  
By default, MAVROS is installed/compiled and installed.  To disable this, set localconf parameter:  
`"maverick_ros::module_mavros": false`  
By default, ROS OpenCV module is disabled as it is a large extra compile.  To enable set localconf parameter:  
`"maverick_ros::module_opencv": true`  

## Running ROS
ROS and MAVROS instances are created by other modules, in their respective environments.  These instances - Maverick sets up Dev and SITL instances - have the necessary services and configuration automatically generated and installed, and are controller through the `maverick` command.  To start or stop ROS:  
`maverick start rosmaster@fc` or `maverick start rosmaster@sitl`  
`maverick stop rosmaster@fc` or `maverick stop rosmaster@sitl`  
To enable or disable at boot:  
`maverick enable rosmaster@fc` or `maverick enable rosmaster@sitl`  
`maverick disable rosmaster@sitl` or `maverick disable rosmaster@sitl`  
Similar commands as above exist to control MAVROS, replacing 'rosmaster' with 'mavros' (eg. `maverick start mavros@fc`).
