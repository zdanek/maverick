class maverick_dev::sitl (
    $sitl_dronekit_source = "http://github.com/dronekit/dronekit-python.git",
    $mavlink_proxy = "mavproxy",
    $mavlink_in = "/dev/ttyAMA0",
    $mavlink_udpinaddress = "",
    $mavlink_udpinport = "",
    $mavlink_active = true,
    $sitl_state = undef,
) {
    
    # Install a virtual environment for dronekit sitl
    file { "/srv/maverick/code/sitl":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => 755,
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
    
    # Install dronekit into sitl
    install_python_module { 'pip-dronekit-sitl':
        pkgname     => 'dronekit',
        virtualenv  => '/srv/maverick/.virtualenvs/sitl',
        ensure      => present,
        owner       => 'mav',
        timeout     => 0,
    }
    install_python_module { 'pip-dronekit-sitl-sitl':
        pkgname     => 'dronekit-sitl',
        virtualenv  => '/srv/maverick/.virtualenvs/sitl',
        ensure      => present,
        owner       => 'mav',
        timeout     => 0,
    }
    install_python_module { 'pip-mavproxy-sitl':
        pkgname     => 'mavproxy',
        virtualenv  => '/srv/maverick/.virtualenvs/sitl',
        ensure      => present,
        owner       => 'mav',
        timeout     => 0,
    }
        
    # This is needed for sitl run
    file { "/var/APM":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => 755,
    }

    file { "/srv/maverick/var/log/sitl":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => 755,
    } ->
    file { "/srv/maverick/var/log/mavlink-sitl":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => 755,
    }
    
    ### make build is out of date, needs updating, but should be rarely used now
    if getvar("maverick_dev::ardupilot::ardupilot_buildsystem") == "make" {
        file { "/etc/systemd/system/maverick-sitl.service":
            content     => template("maverick_dev/maverick-sitl.service.erb"),
            owner       => "root",
            group       => "root",
            mode        => 644,
            notify      => [ Exec["maverick-systemctl-daemon-reload"], Service["maverick-sitl"] ]
        } ->
        service { "maverick-sitl":
            ensure      => $sitl_state,
            enable      => true,
            require     => [ Install_python_module['pip-mavproxy-sitl'], Exec["maverick-systemctl-daemon-reload"] ],
        }
        file { "/srv/maverick/data/config/mavproxy-sitl.screen.conf":
            ensure      => present,
            owner       => "mav",
            group       => "mav",
            source      => "puppet:///modules/maverick_dev/mavproxy-sitl.screen.conf",
            notify      => Service["mavproxy-sitl"]
        } ->
        file { "/srv/maverick/var/log/mavproxy-sitl":
            ensure      => directory,
            owner       => "mav",
            group       => "mav",
            mode        => 755,
        } ->
        file { "/srv/maverick/data/config/mavproxy-sitl.conf":
            ensure      => present,
            owner       => "mav",
            group       => "mav",
            replace     => false, # initialize but don't overwrite in the future
            source      => "puppet:///modules/maverick_dev/mavproxy-sitl.conf",
            notify      => Service["mavproxy-sitl"]
        } ->
        file { "/srv/maverick/software/maverick/bin/mavproxy-sitl.sh":
            ensure      => link,
            target      => "/srv/maverick/software/maverick/manifests/maverick-modules/maverick_dev/files/mavproxy-sitl.sh",
        } ->
        file { "/etc/systemd/system/mavproxy-sitl.service":
            content     => template("maverick_dev/mavproxy-sitl.service.erb"),
            owner       => "root",
            group       => "root",
            mode        => 644,
            notify      => [ Exec["maverick-systemctl-daemon-reload"], Service["mavproxy-sitl"] ]
        } ->
        service { "mavproxy-sitl":
            ensure      => $mavproxy_state,
            enable      => true,
            require       => [ Exec["maverick-systemctl-daemon-reload"], Service["maverick-sitl"] ],
        }

    } elsif getvar("maverick_dev::ardupilot::ardupilot_buildsystem") == "waf" {
        $ardupilot_vehicle = getvar("maverick_dev::ardupilot::ardupilot_vehicle")
        if $ardupilot_vehicle == "copter" {
            $ardupilot_type = "ArduCopter"
        } elsif $ardupilot_vehicle == "plane" {
            $ardupilot_type = "ArduPlane"
        } elsif $ardupilot_vehicle == "rover" {
            $ardupilot_type = "ArduRover"
        }

        file { "/etc/systemd/system/maverick-sitl.service":
            content     => template("maverick_dev/maverick-sitl.service.erb"),
            owner       => "root",
            group       => "root",
            mode        => 644,
            notify      => [ Exec["maverick-systemctl-daemon-reload"], Service["maverick-sitl"] ]
        } ->
        service { "maverick-sitl":
            ensure      => $sitl_state,
            enable      => true,
            require     => [ Install_python_module['pip-mavproxy-sitl'], Exec["maverick-systemctl-daemon-reload"] ],
        }

    }

    if $mavlink_proxy == "mavproxy" {
        maverick_mavlink::cmavnode { "sitl":
            active      => false
        } ->
        maverick_mavlink::mavlinkrouter { "sitl":
            active      => false
        } ->
        maverick_mavlink::mavproxy { "sitl":
            input       => "tcp:localhost:5760",
            instance    => 1,
            startingudp => 14560,
            startingtcp => 5770,
            active      => $mavlink_active,
        }
    } elsif $mavlink_proxy == "cmavnode" {
        maverick_mavlink::mavproxy { "sitl":
            active      => false
        } ->
        maverick_mavlink::mavlinkrouter { "sitl":
            active      => false
        } ->
        maverick_mavlink::cmavnode { "sitl":
            # input       => "utcp:localhost:5760",
            udpinaddress => $mavlink_udpinaddress,
            udpinport   => $mavlink_udpinport,
            startingudp => 14560,
            startingtcp => 5770,
            active      => $mavlink_active,
        }
    } elsif $mavlink_proxy == "mavlinkrouter" {
        maverick_mavlink::cmavnode { "sitl":
            active      => false
        } ->
        maverick_mavlink::mavproxy { "sitl":
            active      => false
        } ->
        maverick_mavlink::mavlinkrouter { "sitl":
            input       => "udp:localhost:5501",
            startingudp => 14560,
            startingtcp => 5770,
            active      => $mavlink_active,
        }
    }

}