class maverick_fc (
    $fc_dronekit_source = "http://github.com/dronekit/dronekit-python.git",
    $mavlink_proxy = "mavproxy",
    $mavlink_active = true,
    $mavlink_input = "/dev/ttyAMA0",
    $mavros_active = true,
    $dflogger_active = false,
    $dflogger_port = 14570,
) {

    # Install a virtual environment for dronekit fc
    file { "/srv/maverick/code/fc":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => 755,
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
    
    install_python_module { 'pip-dronekit-fc':
        pkgname     => 'dronekit',
        virtualenv  => '/srv/maverick/.virtualenvs/fc',
        ensure      => present,
        owner       => 'mav',
        timeout     => 0,
    }
    install_python_module { 'pip-mavproxy-fc':
        pkgname     => 'MAVProxy',
        virtualenv  => '/srv/maverick/.virtualenvs/fc',
        ensure      => present,
        owner       => 'mav',
        timeout     => 0,
    }
        
    file { "/srv/maverick/var/log/mavlink-fc":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => 755,
    }
   
    ### Setup mavlink proxy
    if $mavlink_proxy == "mavproxy" {
        maverick_mavlink::cmavnode { "fc":
            inputaddress => $mavlink_input,
            startingudp => 14570,
            startingtcp => 5780,
            active      => false
        } ->
        maverick_mavlink::mavlink_router { "fc":
            inputtype   => "serial",
            inputaddress => $mavlink_input,
            startingudp => 14570,
            startingtcp => 5780,
            active      => false
        } ->
        maverick_mavlink::mavproxy { "fc":
            inputaddress => $mavlink_input,
            instance    => 2,
            startingudp => 14570,
            startingtcp => 5780,
            active      => $mavlink_active,
        }
    } elsif $mavlink_proxy == "cmavnode" {
        maverick_mavlink::mavproxy { "fc":
            inputaddress => $mavlink_input,
            instance    => 2,
            startingudp => 14570,
            startingtcp => 5780,
            active      => false,
        } ->
        maverick_mavlink::mavlink_router { "fc":
            inputtype   => "serial",
            inputaddress => $mavlink_input,
            startingudp => 14570,
            startingtcp => 5780,
            active      => false
        } ->
        maverick_mavlink::cmavnode { "fc":
            inputaddress => $mavlink_input,
            startingudp => 14570,
            startingtcp => 5780,
            active      => $mavlink_active,
        }
    } elsif $mavlink_proxy == "mavlink-router" {
        maverick_mavlink::cmavnode { "fc":
            inputaddress => $mavlink_input,
            startingudp => 14570,
            startingtcp => 5780,
            active      => false
        } ->
        maverick_mavlink::mavproxy { "fc":
            inputaddress => $mavlink_input,
            instance    => 2,
            startingudp => 14570,
            startingtcp => 5780,
            active      => false
        } ->
        maverick_mavlink::mavlink_router { "fc":
            inputtype   => "serial",
            inputaddress => $mavlink_input,
            startingudp => 14570,
            startingtcp => 5780,
            active      => $mavlink_active,
        }
    }
    
    # Add a mavros service for FC link
    if $mavros_active == true {
        file { "/etc/systemd/system/maverick-mavros-fc.service":
            content     => template("maverick_fc/maverick-mavros-fc.service.erb"),
            owner       => "root",
            group       => "root",
            mode        => 644,
            notify      => Exec["maverick-systemctl-daemon-reload"],
        } ->
        service { "maverick-mavros-fc":
            ensure      => running,
            enable      => true,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/profile.d/30-ros-env.sh"] ]
        }
    } else {
        service { "maverick-mavros-fc":
            ensure      => stopped,
            enable      => false,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/profile.d/30-ros-env.sh"] ]
        }

    }
    
    file { "/srv/maverick/data/config/mavlink/dataflash_logger.conf":
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
        mode        => 644,
        notify      => Exec["maverick-systemctl-daemon-reload"],
    }

    if $dflogger_active {
        service { "maverick-dflogger":
            ensure      => running,
            enable      => true,
            require     => [ Exec["install-dronekit-la"], File["/etc/systemd/system/maverick-dflogger.service"] ],
        }
    } else {
        service { "maverick-dflogger":
            ensure      => stopped,
            enable      => false,
        }
    }

}