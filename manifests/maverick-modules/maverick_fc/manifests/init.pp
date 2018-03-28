class maverick_fc (
    $fc_dronekit_source = "http://github.com/dronekit/dronekit-python.git",
    $mavlink_proxy = "mavlink-router",
    $mavlink_active = true,
    $mavlink_input = "/dev/ttyAMA0",
    $mavlink_inbaud = 115200,
    $mavlink_inflow = false,
    $mavlink_flow = false,
    $mavlink_startingtcp = 5770,
    $mavlink_tcpports = 3,
    $mavlink_startingudp = 14570,
    $mavlink_udpports = 3,
    $mavlink_udpinports = 3,
    $mavlink_serialout = undef,
    $mavlink_outbaud = 115200,
    $mavlink_outflow = false,
    $ros_instance = true,
    $rosmaster_active = true,
    $rosmaster_port = "11311",
    $mavros_active = true,
    $mavros_startup_delay = 10,
    $mavlink_port = "5770",
    $dflogger_active = false,
    $dflogger_port = 14570,
) {

    # Install a virtual environment for dronekit fc
    file { "/srv/maverick/code/fc":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    } ->
    python::virtualenv { '/srv/maverick/code/fc':
        ensure       => present,
        version      => 'system',
        systempkgs   => false,
        distribute   => true,
        venv_dir     => '/srv/maverick/.virtualenvs/fc',
        owner        => 'mav',
        group        => 'mav',
        cwd          => '/srv/maverick/code/fc',
        timeout      => 0,
    } ->
    file { "/srv/maverick/.virtualenvs/fc/lib/python2.7/no-global-site-packages.txt":
        ensure  => absent
    } ->
    oncevcsrepo { "git-fc-dronekit-python":
        gitsource   => $fc_dronekit_source,
        dest        => "/srv/maverick/code/fc/dronekit-python",
    }
    /*
    install_python_module { 'pip-dronekit-fc':
        pkgname     => 'dronekit',
        virtualenv  => '/srv/maverick/.virtualenvs/fc',
        ensure      => present,
        owner       => 'mav',
        timeout     => 0,
        env         => "fc",
    }
    install_python_module { 'pip-mavproxy-fc':
        pkgname     => 'MAVProxy',
        virtualenv  => '/srv/maverick/.virtualenvs/fc',
        ensure      => present,
        owner       => 'mav',
        timeout     => 0,
        require     => Package["python-lxml", "libxml2-dev", "libxslt1-dev"],
        env         => "fc",
    }
    */        
    file { "/srv/maverick/var/log/mavlink-fc":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    }
    
    # Install default params config
    file { "/srv/maverick/config/mavlink/mavlink_params-fc.json":
        ensure      => absent,
    }

    ### Setup mavlink proxy
    if $mavlink_proxy == "mavproxy" {
        maverick_mavlink::cmavnode { "fc":
            inputaddress => $mavlink_input,
            inputbaud   => $mavlink_inbaud,
            inputflow   => $mavlink_inflow,
            startingudp => $mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            outflow     => $mavlink_outflow,
            active      => false,
        } ->
        maverick_mavlink::mavlink_router { "fc":
            inputtype   => "serial",
            inputbaud   => $mavlink_inbaud,
            inputflow   => $mavlink_inflow,
            inputaddress => $mavlink_input,
            startingudp => $mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            outflow     => $mavlink_outflow,
            active      => false,
        } ->
        maverick_mavlink::mavproxy { "fc":
            inputaddress => $mavlink_input,
            inputbaud   => $mavlink_inbaud,
            inputflow   => $mavlink_inflow,
            instance    => 2,
            startingudp => $mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            outflow     => $mavlink_outflow,
            active      => $mavlink_active,
        }
    } elsif $mavlink_proxy == "cmavnode" {
        maverick_mavlink::mavproxy { "fc":
            inputaddress => $mavlink_input,
            inputbaud   => $mavlink_inbaud,
            inputflow   => $mavlink_inflow,
            instance    => 2,
            startingudp => $mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            outflow     => $mavlink_outflow,
            active      => false,
        } ->
        maverick_mavlink::mavlink_router { "fc":
            inputtype   => "serial",
            inputaddress => $mavlink_input,
            inputbaud   => $mavlink_inbaud,
            inputflow   => $mavlink_inflow,
            startingudp => $mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            outflow     => $mavlink_outflow,
            active      => false,
        } ->
        maverick_mavlink::cmavnode { "fc":
            inputaddress => $mavlink_input,
            inputbaud   => $mavlink_inbaud,
            inputflow   => $mavlink_inflow,
            startingudp => $mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            outflow     => $mavlink_outflow,
            active      => $mavlink_active,
        }
    } elsif $mavlink_proxy == "mavlink-router" {
        maverick_mavlink::cmavnode { "fc":
            inputaddress => $mavlink_input,
            inputbaud   => $mavlink_inbaud,
            inputflow   => $mavlink_inflow,
            startingudp => $mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            outflow     => $mavlink_outflow,
            active      => false,
        } ->
        maverick_mavlink::mavproxy { "fc":
            inputaddress => $mavlink_input,
            inputbaud   => $mavlink_inbaud,
            inputflow   => $mavlink_inflow,
            instance    => 2,
            startingudp => $mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            outflow     => $mavlink_outflow,
            active      => false,
        } ->
        maverick_mavlink::mavlink_router { "fc":
            inputtype   => "serial",
            inputaddress => $mavlink_input,
            inputbaud   => $mavlink_inbaud,
            inputflow   => $mavlink_inflow,
            startingudp => $mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            outflow     => $mavlink_outflow,
            active      => $mavlink_active,
        }
    }

    file { "/srv/maverick/config/mavlink/dataflash_logger.conf":
        owner       => "mav",
        group       => "mav",
        mode        => "644",
        content     => template("maverick_fc/dataflash_logger.conf.erb")
    } ->
    # Create a directory for logs
    file { "/srv/maverick/data/logs/dataflash":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    } ->
    file { "/etc/systemd/system/maverick-dflogger.service":
        source      => "puppet:///modules/maverick_fc/maverick-dflogger.service",
        owner       => "root",
        group       => "root",
        mode        => "644",
        notify      => Exec["maverick-systemctl-daemon-reload"],
    }

    if $dflogger_active {
        service_wrapper { "maverick-dflogger":
            ensure      => running,
            enable      => true,
            require     => [ Exec["install-dronekit-la"], File["/etc/systemd/system/maverick-dflogger.service"] ],
        }
    } else {
        service_wrapper { "maverick-dflogger":
            ensure      => stopped,
            enable      => false,
        }
    }
    
    # maverick_fc::ros_instance allows ros to be completely optional
    if $ros_instance == true {
        # Add a ROS master for FC
        maverick_ros::rosmaster { "fc":
            active  => $rosmaster_active,
            port    => $rosmaster_port,
        } ->
        maverick_ros::mavros { "fc":
            active              => $mavros_active,
            rosmaster_port      => $rosmaster_port,
            mavlink_port        => $mavlink_port,
            mavros_startup_delay => $mavros_startup_delay,
        }
    }
    
}