define maverick_dev::apsitl (
    $sitl_dronekit_source = "http://github.com/dronekit/dronekit-python.git",
    $instance_name = "dev",
    $instance_number = 0,
    $sitl_active = true,
    $sitl_port = 5000,
    $rcin_port = 5500,
    $vehicle_type = "copter",
    $vehicle_frame = undef,
    $vehicle_paramfile = undef,
    $mavlink_proxy = "mavlink-router",
    $mavlink_active = true,
    $mavlink_startingtcp = 6000,
    $mavlink_tcpports = 3,
    $mavlink_startingudp = 14000,
    $mavlink_udpports = 3,
    $mavlink_udpinports = 3,
    $mavlink_serialout = undef,
    $mavlink_outbaud = 115200,
    $mavlink_outflow = false,
    $ros_instance = true,
    $rosmaster_active = true,
    $rosmaster_port = 11000,
    $mavros_active = true,
    $mavros_startup_delay = 10,
    $api_instance = true,
    $api_active = true,
    $api_port = 7000
) {
    
    # This class creates an instance of ArduPilot SITL.  It is called by other classes to create an SITL, eg. maverick_dev::apsitl_dev

    # Install a virtual environment for dronekit sitl
    # NB: The python virtualenv is disabled as it is probably overly-complicated and unnecessary.
    # The code remains in case it is useful as an option in the future.
    /*
    file { "/srv/maverick/code/sitl":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    } ->
    python::virtualenv { '/srv/maverick/code/sitl':
        ensure       => present,
        version      => 'system',
        systempkgs   => false,
        distribute   => true,
        venv_dir     => '/srv/maverick/.virtualenvs/sitl',
        owner        => 'mav',
        group        => 'mav',
        cwd          => '/srv/maverick/code/sitl',
        timeout      => 0,
    } ->
    file { "/srv/maverick/.virtualenvs/sitl/lib/python2.7/no-global-site-packages.txt":
        ensure  => absent
    } ->
    oncevcsrepo { "git-sitl-dronekit-python":
        gitsource   => $sitl_dronekit_source,
        dest        => "/srv/maverick/code/sitl/dronekit-python",
    }
    
    install_python_module { 'pip-dronekit-sitl':
        pkgname     => 'dronekit-sitl',
        ensure      => present,
        timeout     => 0,
    }
    */

    /*
    # Install dronekit into sitl
    install_python_module { 'pip-dronekit-sitl':
        pkgname     => 'dronekit',
        virtualenv  => '/srv/maverick/.virtualenvs/sitl',
        ensure      => present,
        owner       => 'mav',
        timeout     => 0,
        env         => "sitl",
    }
    install_python_module { 'pip-dronekit-sitl-sitl':
        pkgname     => 'dronekit-sitl',
        virtualenv  => '/srv/maverick/.virtualenvs/sitl',
        ensure      => present,
        owner       => 'mav',
        timeout     => 0,
        env         => "sitl",
    }
    install_python_module { 'pip-mavproxy-sitl':
        pkgname     => 'MAVProxy',
        virtualenv  => '/srv/maverick/.virtualenvs/sitl',
        ensure      => present,
        owner       => 'mav',
        timeout     => 0,
        require     => Package["python-lxml", "libxml2-dev", "libxslt1-dev"],
        env         => "sitl",
    }
    */

    file { [ "/srv/maverick/var/log/dev/apsitl_${instance_name}", "/srv/maverick/data/dev/mavlink/apsitl_${instance_name}" ]:
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
        notify      => Service_wrapper["maverick-apsitl@${instance_name}"],
    }
    file { "/srv/maverick/config/dev/apsitl_${instance_name}.conf":
        content     => template("maverick_dev/apsitl.conf.erb"),
        owner       => "mav",
        group       => "mav",
        mode        => "644",
        replace     => false,
        notify      => Service_wrapper["maverick-apsitl@${instance_name}"],
    } ->
    file { "/srv/maverick/config/mavlink/mavlink_params-apsitl_${instance_name}.json":
        ensure      => absent,
    }
    
    if $sitl_active == true {
        service_wrapper { "maverick-apsitl@${instance_name}":
            ensure      => running,
            enable      => true,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-apsitl@.service"] ],
        }
    } else {
        service_wrapper { "maverick-apsitl@${instance_name}":
            ensure      => stopped,
            enable      => false,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-apsitl@.service"] ],
        }
    }

    if $ros_instance == true {
        $notifyResources = [ Service_wrapper["maverick-apsitl@${instance_name}"], Service["maverick-rosmaster@apsitl_${instance_name}"] ]
    } else {
        $notifyResources = Service_wrapper["maverick-apsitl@${instance_name}"]
    }
    if $mavlink_proxy == "mavproxy" {
        maverick_mavlink::cmavnode { "apsitl_${instance_name}":
            active      => false,
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
        } ->
        maverick_mavlink::mavlink_router { "apsitl_${instance_name}":
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
            active      => false,
        } ->
        maverick_mavlink::mavproxy { "apsitl_${instance_name}":
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
        }
    } elsif $mavlink_proxy == "cmavnode" {
        maverick_mavlink::mavproxy { "apsitl_${instance_name}":
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
            active      => false,
        } ->
        maverick_mavlink::mavlink_router { "apsitl_${instance_name}":
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
            active      => false,
        } ->
        maverick_mavlink::cmavnode { "apsitl_${instance_name}":
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
        }
    } elsif $mavlink_proxy == "mavlink-router" {
        maverick_mavlink::cmavnode { "apsitl_${instance_name}":
            active      => false,
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
        } ->
        maverick_mavlink::mavproxy { "apsitl_${instance_name}":
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
            active      => false,
        } ->
        maverick_mavlink::mavlink_router { "apsitl_${instance_name}":
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
        }
    }

    # maverick_dev::sitl::ros_instance allows ros to be completely optional
    if $ros_instance == true {
        # Add a ROS master for SITL
        maverick_ros::rosmaster { "apsitl_${instance_name}":
            active  => $rosmaster_active,
            port    => $actual_rosmaster_port,
        } ->
        maverick_ros::mavros { "apsitl_${instance_name}":
            active              => $mavros_active,
            rosmaster_port      => $actual_rosmaster_port,
            mavlink_port        => $actual_mavlink_startingtcp,
            mavros_startup_delay => $mavros_startup_delay,
        }
    }

    if $api_instance == true {
        # Create an API instance
        maverick_web::api { "api-apsitl_${instance_name}":
            instance    => "apsitl_${instance_name}",
            active      => $api_active,
            apiport     => $actual_api_port,
            rosport     => $actual_rosmaster_port,
        }
    }

}
