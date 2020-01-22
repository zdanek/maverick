define maverick_dev::apsitl (
    $sitl_dronekit_source = "http://github.com/dronekit/dronekit-python.git",
    $instance_name = "apsitl",
    $instance_number = 0,
    $sitl_active = true,
    $sitl_port = 5000,
    $rcin_port = 5500,
    $vehicle_type = "copter",
    $vehicle_frame = undef,
    $vehicle_paramfile = undef,
    $mavlink_proxy = "mavlink-router",
    $mavlink_active = true,
    $mavlink_logging = false,
    $mavlink_startingtcp = 6000,
    $mavlink_tcpports = 3,
    $mavlink_startingudp = 14000,
    $mavlink_udpports = 3,
    $mavlink_udpinports = 3,
    $mavlink_serialout = undef,
    $mavlink_outbaud = 115200,
    $mavlink_outflow = false,
    $mavlink_replaceconfig = true,
    $ros_instance = true,
    $rosmaster_active = true,
    $rosmaster_port = 11000,
    $mavros_active = true,
    $mavros_startup_delay = 10,
    $api_instance = true,
    $api_active = true,
    $api_port = 7000,
    $api_debug = false,
    $api_devmode = false,
    $api_replaceconfig = true,
    $status_priority = "151",
    $status_entries = true,
) {
    
    # This class creates an instance of ArduPilot SITL.  It is called by other classes to create an SITL, eg. maverick_dev::apsitl

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
    if $sitl_port == 5000 {
        $actual_sitl_port = $sitl_port + ($instance_number * 10)
    } else {
        $actual_sitl_port = $sitl_port
    }    

    if $rcin_port == 5500 {
        $actual_rcin_port = $rcin_port + ($instance_number * 10)
    } else {
        $actual_rcin_port = $rcin_port
    }

    if $mavlink_startingtcp == 6000 {
        $actual_mavlink_startingtcp = $mavlink_startingtcp + ($instance_number * 10)
    } else {
        $mavlink_startingtcp = $mavlink_startingtcp
    }
    
    if $mavlink_startingudp == 14000 {
        $actual_mavlink_startingudp = $mavlink_startingudp + ($instance_number * 10)
    } else {
        $actual_mavlink_startingudp = $mavlink_startingudp
    }

    if $rosmaster_port == 11000 {
        $actual_rosmaster_port = $rosmaster_port + ($instance_number * 10)
    } else {
        $actual_rosmaster_port = $rosmaster_port
    }

    if $api_port == 7000 {
        $actual_api_port = $api_port + ($instance_number * 10)
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

    # maverick_dev::sitl::ros_instance allows ros to be completely optional
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
