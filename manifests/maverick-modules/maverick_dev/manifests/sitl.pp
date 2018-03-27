class maverick_dev::sitl (
    $sitl_dronekit_source = "http://github.com/dronekit/dronekit-python.git",
    $jsbsim_source = "http://github.com/tridge/jsbsim.git",
    $mavlink_proxy = "mavlink-router",
    $mavlink_active = true,
    $mavlink_startingtcp = 5780,
    $mavlink_tcpports = 3,
    $mavlink_startingudp = 14580,
    $mavlink_udpports = 3,
    $mavlink_udpinports = 3,
    $mavlink_serialout = undef,
    $mavlink_outbaud = 115200,
    $ros_instance = true,
    $rosmaster_active = true,
    $rosmaster_port = 11313,
    $mavros_active = true,
    $mavros_startup_delay = 10,
    $mavlink_port = 5780,
    $sitl_active = true,
    $api_instance = true,
    $api_active = false,
) {
    
    # Install a virtual environment for dronekit sitl
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
        
    # This is needed for sitl run
    file { "/var/APM":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    }

    file { "/srv/maverick/var/log/sitl":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    } ->
    file { "/srv/maverick/var/log/mavlink-sitl":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    }
    
    $ardupilot_vehicle = getvar("maverick_dev::ardupilot::ardupilot_vehicle")
    if $ardupilot_vehicle == "copter" {
        $ardupilot_type = "ArduCopter"
    } elsif $ardupilot_vehicle == "plane" {
        $ardupilot_type = "ArduPlane"
    } elsif $ardupilot_vehicle == "rover" {
        $ardupilot_type = "APMrover2"
    } elsif $ardupilot_vehicle == "sub" {
        $ardupilot_type = "ArduSub"
    } elsif $ardupilot_vehicle == "antennatracker" {
        $ardupilot_type = "AntennaTracker"
    } else {
        $ardupilot_type = $ardupilot_vehicle
    }
    
    # If SITL plane, compile jsbsim and install service
    if $ardupilot_vehicle == "plane" and ! ("install_flag_jsbsim" in $installflags) {
        ensure_packages(["libexpat1-dev"])
        oncevcsrepo { "git-jsbsim":
            gitsource   => $jsbsim_source,
            dest        => "/srv/maverick/var/build/jsbsim",
        } ->
        exec { "jsbsim-autogen":
            command     => "/srv/maverick/var/build/jsbsim/autogen.sh --enable-libraries --prefix=/srv/maverick/software/jsbsim --exec-prefix=/srv/maverick/software/jsbsim",
            cwd         => "/srv/maverick/var/build/jsbsim",
            creates     => "/srv/maverick/var/build/jsbsim/Makefile",
            user        => "mav",
            require     => Package["libexpat1-dev"],
        } ->
        exec { "jsbsim-make":
            command     => "/usr/bin/make",
            cwd         => "/srv/maverick/var/build/jsbsim",
            creates     => "/srv/maverick/var/build/jsbsim/src/JSBSim",
            user        => "mav",
        } ->
        exec { "jsbsim-makeinstall":
            command     => "/usr/bin/make install",
            cwd         => "/srv/maverick/var/build/jsbsim",
            creates     => "/srv/maverick/software/jsbsim/bin/JSBSim",
            user        => "mav",
        } ->
        file { "/srv/maverick/software/jsbsim/bin":
            ensure      => directory,
            owner       => "mav",
            group       => "mav",
        } ->
        exec { "jsbsim-cpbin":
            command     => "/bin/cp /srv/maverick/var/build/jsbsim/src/JSBSim /srv/maverick/software/jsbsim/bin",
            creates     => "/srv/maverick/software/jsbsim/bin/JSBSim",
        } ->
        file { "/srv/maverick/var/build/.install_flag_jsbsim":
            ensure      => file,
            owner       => "mav",
            group       => "mav",
            mode        => "644",
        }
        
    }
    
    file { "/srv/maverick/software/maverick/bin/sitl.sh":
        ensure      => link,
        target      => "/srv/maverick/software/maverick/manifests/maverick-modules/maverick_dev/files/sitl.sh",
    } ->
    file { "/srv/maverick/config/dev/sitl.screen.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        content     => template("maverick_dev/sitl.screen.conf.erb"),
        notify      => Service_wrapper["maverick-sitl"],
    }
    file { "/srv/maverick/config/dev/sitl.conf":
        source      => "puppet:///modules/maverick_dev/sitl.conf",
        owner       => "mav",
        group       => "mav",
        mode        => "644",
        replace     => false,
        notify      => Service_wrapper["maverick-sitl"],
    } ->
    file { "/srv/maverick/config/dev/sitl-vehicle.conf":
        content     => "VEHICLE_TYPE=${ardupilot_type}",
        owner       => "mav",
        group       => "mav",
        mode        => "644",
        notify      => Service_wrapper["maverick-sitl"],
    } ->
    file { "/etc/systemd/system/maverick-sitl.service":
        content     => template("maverick_dev/maverick-sitl.service.erb"),
        owner       => "root",
        group       => "root",
        mode        => "644",
        notify      => [ Exec["maverick-systemctl-daemon-reload"], Service_wrapper["maverick-sitl"] ]
    } ->
    file { "/srv/maverick/config/mavlink/mavlink_params-sitl.json":
        ensure      => absent,
    }
    
    if $sitl_active == true {
        service_wrapper { "maverick-sitl":
            ensure      => running,
            enable      => true,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-sitl.service"] ],
        }
    } else {
        service_wrapper { "maverick-sitl":
            ensure      => stopped,
            enable      => false,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-sitl.service"] ],
        }
    }

    if $ros_instance == true {
        $notifyResources = [ Service_wrapper["maverick-sitl"], Service_wrapper["maverick-rosmaster@sitl"] ]
    } else {
        $notifyResources = Service_wrapper["maverick-sitl"]
    }
    if $mavlink_proxy == "mavproxy" {
        maverick_mavlink::cmavnode { "sitl":
            active      => false,
            inputaddress => "tcp:localhost:5760", # Note cmavnode doesn't support sitl/tcp yet
            startingudp => $mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            notify      => $notifyResources,
        } ->
        maverick_mavlink::mavlink_router { "sitl":
            inputtype   => "tcp",
            inputaddress => "127.0.0.1",
            inputport   => "5760",
            startingudp => $mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            active      => false,
        } ->
        maverick_mavlink::mavproxy { "sitl":
            inputaddress => "tcp:localhost:5760",
            instance    => 1,
            startingudp => $mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            active      => $mavlink_active,
            notify      => $notifyResources,
        }
    } elsif $mavlink_proxy == "cmavnode" {
        maverick_mavlink::mavproxy { "sitl":
            inputaddress => "tcp:localhost:5760",
            instance    => 1,
            startingudp => $mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            active      => false,
        } ->
        maverick_mavlink::mavlink_router { "sitl":
            inputtype   => "tcp",
            inputaddress => "127.0.0.1",
            inputport   => "5760",
            startingudp => $mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            active      => false,
        } ->
        maverick_mavlink::cmavnode { "sitl":
            inputaddress => "tcp:localhost:5760", # Note cmavnode doesn't support sitl/tcp yet
            startingudp => $mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            active      => $mavlink_active,
            notify      => $notifyResources,
        }
    } elsif $mavlink_proxy == "mavlink-router" {
        maverick_mavlink::cmavnode { "sitl":
            active      => false,
            inputaddress => "tcp:localhost:5760", # Note cmavnode doesn't support sitl/tcp yet
            startingudp => $mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            notify      => $notifyResources,
        } ->
        maverick_mavlink::mavproxy { "sitl":
            inputaddress => "tcp:localhost:5760",
            instance    => 1,
            startingudp => $mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            active      => false,
        } ->
        maverick_mavlink::mavlink_router { "sitl":
            inputtype   => "tcp",
            inputaddress => "127.0.0.1",
            inputport   => "5760",
            startingudp => $mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            active      => $mavlink_active,
            notify      => $notifyResources,
        }
    }

    # maverick_dev::sitl::ros_instance allows ros to be completely optional
    if $ros_instance == true {
        # Add a ROS master for SITL
        maverick_ros::rosmaster { "sitl":
            active  => $rosmaster_active,
            port    => $rosmaster_port,
        } ->
        maverick_ros::mavros { "sitl":
            active              => $mavros_active,
            rosmaster_port      => $rosmaster_port,
            mavlink_port        => $mavlink_port,
            mavros_startup_delay => $mavros_startup_delay,
        }
    }

    if $api_instance == true {
        # Create an API instance
        maverick_web::api { "api-sitl":
            instance    => "sitl",
            active      => $api_active,
            apiport     => 6801,
            rosport     => $rosmaster_port,
        }
    }

}