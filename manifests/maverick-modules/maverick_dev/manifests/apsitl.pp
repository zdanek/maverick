# @summary
#   This function creates an instance of ArduPilot SITL.  It is called by other classes to create an SITL, eg. maverick_dev::apsitl
#
# @example
#   @@maverick_dev::apsitl { $instance_name:
#        instance_name       => "test_sitl",
#        instance_number     => 3,
#        sitl_active         => true,
#        rosmaster_active    => false,
#        mavlink_proxy       => 'mavproxy',
#        mavros_active       => false,
#        mavlink_active      => true,
#        mavlink_replaceconfig => true,
#        status_priority     => "154",
#        api_active          => false,
#        api_devmode         => false,
#        api_debug           => false,
#        api_replaceconfig   => true,
#    }
#
# @param instance_name
#   Name of the SITL instance - this is a short name used to generate file/service names, eg. 'apsitl'
# @param instance_number
#   This must be a unique integer, used to identify the instance
# @param sitl_active
#   If true, the SITL service will be activated and enabled at boot time
# @param sitl_port
#   Port number that SITL listens onlyif
# @param rcin_port
#   Port number that SITL RC in listens on
# @param vehicle_type
#   Type of Ardupilot vehicle to use, eg. copter, plane, sub etc
# @param vehicle_frame
#   Frame type - eg. quad, heli, plane, quadplane, rover
# @param vehicle_paramfile
#   Specify an optional parameter file to override the default
# @param mavlink_proxy
#   Type of mavlink proxy to use, eg. mavlink-router, mavproxy or cmavnode
# @param mavlink_active
#   If true, the mavlink proxy will be activated and enabled at boot time
# @param mavlink_logging
#   If true and supported by the mavlink proxy software, then mavlink data will be logged to file
# @param mavlink_startingtcp
#   Start TCP port - if more than one port then each one will be incremented from this starting value
# @param mavlink_tcpports
#   Number of TCP ports that the mavlink proxy will listen on
# @param mavlink_startingudp
#   Start UDP port - if more than one port then each one will be incremented from this starting value
# @param mavlink_udpports
#   Number of UDP ports to listen on
# @param mavlink_udpinports
#   Number of UDP In ports to listen on
# @param mavlink_serialout
#   If set, proxy mavlink data out on this serial port
# @param mavlink_outbaud
#   Baud rate of mavlink_serialout port
# @param mavlink_outflow
#   If mavlink_serialout port should use hardware flow control
# @param mavlink_replaceconfig
#   If set to true, this will overwrite the mavlink proxy config file.  If false, edits can be made directly to the config file without being managed/overwritten
# @param ros_instance
#   If true, create a separate ROS instance for this SITL instance
# @param rosmaster_active
#   If true, set this separate ROS instance active and enabled at boot time
# @param rosmaster_port
#   Define the port number that the ROS master will listen on.  This must be unique across all ROS instances.
# @param mavros_active
#   If true, the separate MAVROS instance will be activated and enabled at boot time
# @param mavros_startup_delay
#   This delay causes Mavros to wait before starting, to give ROS and SITL time to boot fully first.  Should be increased on slower boards/environments.
# @param api_instance
#   If true, create a separate maverick-api instance
# @param api_name
#   Descriptive name of api instance (keep short)
# @param api_active
#   If true, this maverick-api instance will be activated and enabled at boot time
# @param api_port
#   Port number that the separate maverick-api instance will listen on.  Must be unique across all -api instances.
# @param api_debug
#   If true, turn on the -api debug mode
# @param api_devmode
#   If true, turn on the -api dev mode
# @param api_replaceconfig
#   If set to true, this will overwrite the -api config file.  If false, edits can be made directly to the config file without being managed/overwritten
# @param status_priority
#   This defines the priority of maverick status.d entry for this instance.  This determines the display order in `maverick status`
# @param status_entries
#
define maverick_dev::apsitl (
    String $instance_name = "apsitl",
    Integer $instance_number = 0,
    Boolean $sitl_active = true,
    Integer $sitl_port = 6500,
    Integer $rcin_port = 6501,
    String $vehicle_type = "copter",
    Optional[String] $vehicle_frame = undef,
    Optional[String] $vehicle_paramfile = undef,
    String $mavlink_proxy = "mavlink-router",
    Boolean $mavlink_active = true,
    Boolean $mavlink_logging = false,
    Integer $mavlink_startingtcp = 6504,
    Integer $mavlink_tcpports = 3,
    Integer $mavlink_startingudp = 6507,
    Integer $mavlink_udpports = 3,
    Integer $mavlink_udpinports = 3,
    Optional[String] $mavlink_serialout = undef,
    Integer $mavlink_outbaud = 115200,
    Boolean $mavlink_outflow = false,
    Boolean $mavlink_replaceconfig = true,
    Boolean $ros_instance = true,
    Boolean $rosmaster_active = true,
    Integer $rosmaster_port = 6502,
    Boolean $mavros_active = true,
    Integer $mavros_startup_delay = 10,
    Boolean $api_instance = true,
    String $api_name = "",
    Boolean $api_active = true,
    Integer $api_port = 6503,
    Boolean $api_debug = false,
    Boolean $api_devmode = false,
    Boolean $api_replaceconfig = true,
    String $status_priority = "151",
    Boolean $status_entries = true,
) {
    file { [ "/srv/maverick/var/log/dev/${instance_name}", "/srv/maverick/data/dev/mavlink/apsitl_${instance_name}" ]:
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    }
    
    if $vehicle_type == "copter" {
        $ardupilot_type = "ArduCopter"
        if !$vehicle_frame {
            $ardupilot_frame = "quad"
        } else {
            $ardupilot_frame = $vehicle_frame
        }
        if !$vehicle_paramfile {
            $ardupilot_paramfile = "copter"
        } else {
            $ardupilot_paramfile = $vehicle_paramfile
        }

    } elsif $vehicle_type == "plane" {
        $ardupilot_type = "ArduPlane"
        if !$vehicle_frame {
            $ardupilot_frame = "plane"
        } else {
            $ardupilot_frame = $vehicle_frame
        }
        if !$vehicle_paramfile {
            $ardupilot_paramfile = "plane"
        } else {
            $ardupilot_paramfile = $vehicle_paramfile
        }
    } elsif $vehicle_type == "rover" {
        $ardupilot_type = "APMrover2"
        if !$vehicle_frame {
            $ardupilot_frame = "rover"
        } else {
            $ardupilot_frame = $vehicle_frame
        }
        if !$vehicle_paramfile {
            $ardupilot_paramfile = "rover"
        } else {
            $ardupilot_paramfile = $vehicle_paramfile
        }
    } elsif $vehicle_type == "sub" {
        $ardupilot_type = "ArduSub"
        if !$vehicle_frame {
            $ardupilot_frame = "vectored"
        } else {
            $ardupilot_frame = $vehicle_frame
        }
        if !$vehicle_paramfile {
            $ardupilot_paramfile = "sub"
        } else {
            $ardupilot_paramfile = $vehicle_paramfile
        }
    } elsif $vehicle_type == "antennatracker" {
        $ardupilot_type = "AntennaTracker"
        if !$vehicle_frame {
            $ardupilot_frame = "tracker"
        } else {
            $ardupilot_frame = $vehicle_frame
        }
        if !$vehicle_paramfile {
            $ardupilot_paramfile = "tracker"
        } else {
            $ardupilot_paramfile = $vehicle_paramfile
        }
    } else {
        $ardupilot_type = $vehicle_type
        $ardupilot_frame = $vehicle_frame
        $ardupilot_paramfile = $vehicle_paramfile
    }
    
    # Calculate actual SITL ports from instance multiplier, unless the ports have been specifically set
    if $sitl_port == 6500 {
        $actual_sitl_port = $sitl_port + ($instance_number * 20)
    } else {
        $actual_sitl_port = $sitl_port
    }    

    if $rcin_port == 6501 {
        $actual_rcin_port = $rcin_port + ($instance_number * 20)
    } else {
        $actual_rcin_port = $rcin_port
    }

    if $mavlink_startingtcp == 6504 {
        $actual_mavlink_startingtcp = $mavlink_startingtcp + ($instance_number * 20)
    } else {
        $actual_mavlink_startingtcp = $mavlink_startingtcp
    }
    
    if $mavlink_startingudp == 6507 {
        $actual_mavlink_startingudp = $mavlink_startingudp + ($instance_number * 20)
    } else {
        $actual_mavlink_startingudp = $mavlink_startingudp
    }

    if $rosmaster_port == 6502 {
        $actual_rosmaster_port = $rosmaster_port + ($instance_number * 20)
    } else {
        $actual_rosmaster_port = $rosmaster_port
    }

    if $api_port == 6503 {
        $actual_api_port = $api_port + ($instance_number * 20)
    } else {
        $actual_api_port = $api_port
    }

    file { "/srv/maverick/config/dev/apsitl_${instance_name}.screen.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        content     => template("maverick_dev/apsitl.screen.conf.erb"),
        notify      => Service["maverick-apsitl@${instance_name}"],
    }
    file { "/srv/maverick/config/dev/apsitl_${instance_name}.conf":
        content     => template("maverick_dev/apsitl.conf.erb"),
        owner       => "mav",
        group       => "mav",
        mode        => "644",
        replace     => false,
        notify      => Service["maverick-apsitl@${instance_name}"],
    } ->
    file { "/srv/maverick/config/mavlink/mavlink_params-${instance_name}.json":
        ensure      => absent,
    }
    
    if $sitl_active == true {
        service { "maverick-apsitl@${instance_name}":
            ensure      => running,
            enable      => true,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-apsitl@.service"] ],
        }
    } else {
        service { "maverick-apsitl@${instance_name}":
            ensure      => stopped,
            enable      => false,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-apsitl@.service"] ],
        }
    }

    if $ros_instance == true {
        $notifyResources = [ Service["maverick-apsitl@${instance_name}"], Service["maverick-rosmaster@${instance_name}"] ]
    } else {
        $notifyResources = Service["maverick-apsitl@${instance_name}"]
    }
    if $mavlink_proxy == "mavproxy" {
        maverick_mavlink::cmavnode { "${instance_name}":
            inputaddress => "tcp:localhost:${actual_sitl_port}", # Note cmavnode doesn't support sitl/tcp yet
            startingudp => $actual_mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $actual_mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            outflow     => $mavlink_outflow,
            notify      => $notifyResources,
            replaceconfig => $mavlink_replaceconfig,
        } ->
        maverick_mavlink::mavlink_router { "${instance_name}":
            inputtype   => "tcp",
            inputaddress => "127.0.0.1",
            inputport   => $actual_sitl_port,
            startingudp => $actual_mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $actual_mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            outflow     => $mavlink_outflow,
            logging     => $mavlink_logging,
            replaceconfig => $mavlink_replaceconfig,
        } ->
        maverick_mavlink::mavproxy { "${instance_name}":
            inputaddress => "tcp:localhost:${actual_sitl_port}",
            instance    => 1,
            startingudp => $actual_mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $actual_mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            outflow     => $mavlink_outflow,
            active      => $mavlink_active,
            notify      => $notifyResources,
            replaceconfig => $mavlink_replaceconfig,
        }
    } elsif $mavlink_proxy == "cmavnode" {
        maverick_mavlink::mavproxy { "${instance_name}":
            inputaddress => "tcp:localhost:${actual_sitl_port}",
            instance    => 1,
            startingudp => $actual_mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $actual_mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            outflow     => $mavlink_outflow,
            replaceconfig => $mavlink_replaceconfig,
        } ->
        maverick_mavlink::mavlink_router { "${instance_name}":
            inputtype   => "tcp",
            inputaddress => "127.0.0.1",
            inputport   => $actual_sitl_port,
            startingudp => $actual_mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $actual_mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            outflow     => $mavlink_outflow,
            logging     => $mavlink_logging,
            replaceconfig => $mavlink_replaceconfig,
        } ->
        maverick_mavlink::cmavnode { "${instance_name}":
            inputaddress => "tcp:localhost:${actual_sitl_port}", # Note cmavnode doesn't support sitl/tcp yet
            startingudp => $actual_mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $actual_mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            outflow     => $mavlink_outflow,
            active      => $mavlink_active,
            notify      => $notifyResources,
            replaceconfig => $mavlink_replaceconfig,
        }
    } elsif $mavlink_proxy == "mavlink-router" {
        maverick_mavlink::cmavnode { "${instance_name}":
            inputaddress => "tcp:localhost:${actual_sitl_port}", # Note cmavnode doesn't support sitl/tcp yet
            startingudp => $actual_mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $actual_mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            outflow     => $mavlink_outflow,
            notify      => $notifyResources,
            replaceconfig => $mavlink_replaceconfig,
        } ->
        maverick_mavlink::mavproxy { "${instance_name}":
            inputaddress => "tcp:localhost:${actual_sitl_port}",
            instance    => 1,
            startingudp => $actual_mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $actual_mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            outflow     => $mavlink_outflow,
            replaceconfig => $mavlink_replaceconfig,
        } ->
        maverick_mavlink::mavlink_router { "${instance_name}":
            inputtype   => "tcp",
            inputaddress => "127.0.0.1",
            inputport   => $actual_sitl_port,
            startingudp => $actual_mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $actual_mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            outflow     => $mavlink_outflow,
            active      => $mavlink_active,
            notify      => $notifyResources,
            logging     => $mavlink_logging,
            replaceconfig => $mavlink_replaceconfig,
        }
    }

    # maverick_dev::apsitl_dev::ros_instance allows ros to be completely optional
    if $ros_instance == true {
        # Add a ROS master for SITL
        maverick_ros::rosmaster { "${instance_name}":
            active  => $rosmaster_active,
            port    => $actual_rosmaster_port,
        } ->
        maverick_ros::mavros { "${instance_name}":
            active              => $mavros_active,
            rosmaster_port      => $actual_rosmaster_port,
            mavlink_port        => $actual_mavlink_startingtcp,
            mavros_startup_delay => $mavros_startup_delay,
        }
        file { "/srv/maverick/software/maverick/bin/status.d/${status_priority}.${instance_name}/102.rosmaster.status":
            owner   => "mav",
            content => "rosmaster@${instance_name},ROS (${instance_name})\n",
        }
        file { "/srv/maverick/software/maverick/bin/status.d/${status_priority}.${instance_name}/103.mavros.status":
            owner   => "mav",
            content => "mavros@${instance_name},MavROS (${instance_name})\n",
        }
    }

    if $api_instance == true {
        # Create an API instance
        maverick_web::api { "api-${instance_name}":
            instance    => "${instance_name}",
            api_name    => $api_name,
            active      => $api_active,
            apiport     => $actual_api_port,
            rosport     => $actual_rosmaster_port,
            devmode     => $api_devmode,
            debug       => $api_debug,
            replaceconfig    => $api_replaceconfig,
        }
        file { "/srv/maverick/software/maverick/bin/status.d/${status_priority}.${instance_name}/104.api.status":
            owner   => "mav",
            content => "api@${instance_name},MavAPI (${instance_name})\n",
        }
    }

    if $status_entries == true {
        # status.d entry for collectd
        file { "/srv/maverick/software/maverick/bin/status.d/${status_priority}.${instance_name}/":
            ensure  => directory,
            owner   => "mav",
            mode    => "755",
        }
        file { "/srv/maverick/software/maverick/bin/status.d/${status_priority}.${instance_name}/__init__":
            owner   => "mav",
            content => "APSITL (${instance_name})\n",
        }
        file { "/srv/maverick/software/maverick/bin/status.d/${status_priority}.${instance_name}/100.apsitl.status":
            owner   => "mav",
            content => "apsitl@${instance_name},AP SITL (${instance_name})\n",
        }
        file { "/srv/maverick/software/maverick/bin/status.d/${status_priority}.${instance_name}/101.mavlink.status":
            owner   => "mav",
            content => "mavlink@${instance_name},Mavlink (${instance_name})\n",
        }        
    }
}
