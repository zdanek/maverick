class maverick_dev::sitl (
    $sitl_dronekit_source = "http://github.com/dronekit/dronekit-python.git",
    $mavlink_proxy = "mavproxy",
    $mavlink_active = true,
    $mavros_active = true,
    $sitl_active = true,
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
    
    $ardupilot_vehicle = getvar("maverick_dev::ardupilot::ardupilot_vehicle")
    if $ardupilot_vehicle == "copter" {
        $ardupilot_type = "ArduCopter"
    } elsif $ardupilot_vehicle == "plane" {
        $ardupilot_type = "ArduPlane"
    } elsif $ardupilot_vehicle == "rover" {
        $ardupilot_type = "ArduRover"
    } else {
        $ardupilot_type = $ardupilot_vehicle
    }
    file { "/etc/systemd/system/maverick-sitl.service":
        content     => template("maverick_dev/maverick-sitl.service.erb"),
        owner       => "root",
        group       => "root",
        mode        => 644,
        notify      => [ Exec["maverick-systemctl-daemon-reload"], Service["maverick-sitl"] ]
    }
    if $sitl_active {
        service { "maverick-sitl":
            ensure      => running,
            enable      => true,
            require     => [ Install_python_module['pip-mavproxy-sitl'], Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-sitl.service"] ],
        }
    } else {
        service { "maverick-sitl":
            ensure      => stopped,
            enable      => false,
            require     => [ Install_python_module['pip-mavproxy-sitl'], Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-sitl.service"] ],
        }
    }

    if $mavlink_proxy == "mavproxy" {
        maverick_mavlink::cmavnode { "sitl":
            active      => false
        } ->
        maverick_mavlink::mavlinkrouter { "sitl":
            inputtype   => "tcp",
            inputaddress => "127.0.0.1",
            inputport   => "5760",
            startingudp => 14560,
            startingtcp => 5770,
            active      => false
        } ->
        maverick_mavlink::mavproxy { "sitl":
            inputaddress => "tcp:localhost:5760",
            instance    => 1,
            startingudp => 14560,
            startingtcp => 5770,
            active      => $mavlink_active,
        }
    } elsif $mavlink_proxy == "cmavnode" {
        maverick_mavlink::mavproxy { "sitl":
            inputaddress => "tcp:localhost:5760",
            instance    => 1,
            startingudp => 14560,
            startingtcp => 5770,
            active      => false
        } ->
        maverick_mavlink::mavlinkrouter { "sitl":
            inputtype   => "tcp",
            inputaddress => "127.0.0.1",
            inputport   => "5760",
            startingudp => 14560,
            startingtcp => 5770,
            active      => false
        } ->
        maverick_mavlink::cmavnode { "sitl":
            inputaddress => "tcp:localhost:5760", # Note cmavnode doesn't support sitl/tcp yet
            startingudp => 14560,
            startingtcp => 5770,
            active      => $mavlink_active,
        }
    } elsif $mavlink_proxy == "mavlinkrouter" {
        maverick_mavlink::cmavnode { "sitl":
            active      => false
        } ->
        maverick_mavlink::mavproxy { "sitl":
            inputaddress => "tcp:localhost:5760",
            instance    => 1,
            startingudp => 14560,
            startingtcp => 5770,
            active      => false
        } ->
        maverick_mavlink::mavlinkrouter { "sitl":
            inputtype   => "tcp",
            inputaddress => "127.0.0.1",
            inputport   => "5760",
            startingudp => 14560,
            startingtcp => 5770,
            active      => $mavlink_active,
        }
    }

    # Add a mavros service for sitl link
    if $mavros_active == true {
        $distribution = getvar("maverick_ros::distribution")
        file { "/etc/systemd/system/maverick-mavros-sitl.service":
            content     => template("maverick_dev/maverick-mavros-sitl.service.erb"),
            owner       => "root",
            group       => "root",
            mode        => 644,
            notify      => Exec["maverick-systemctl-daemon-reload"],
        } ->
        service { "maverick-mavros-sitl":
            ensure      => running,
            enable      => true,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/profile.d/30-ros-env.sh"] ]
        }
    } else {
        service { "maverick-mavros-sitl":
            ensure      => stopped,
            enable      => false,
            require     => [ Exec["maverick-systemctl-daemon-reload"] ]
        }
    }
    
}