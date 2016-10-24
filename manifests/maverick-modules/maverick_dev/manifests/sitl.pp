class maverick_dev::sitl (
    $sitl_dronekit_source = "http://github.com/dronekit/dronekit-python.git",
    $mavproxy_state = undef,
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
    
    # Install dronekit-sitl into sitl
    python::pip { 'pip-dronekit-sitl':
        pkgname     => 'dronekit',
        virtualenv  => '/srv/maverick/.virtualenvs/sitl',
        ensure      => present,
        owner       => 'mav',
        timeout     => 0,
    }
    python::pip { 'pip-dronekit-sitl-sitl':
        pkgname     => 'dronekit-sitl',
        virtualenv  => '/srv/maverick/.virtualenvs/sitl',
        ensure      => present,
        owner       => 'mav',
        timeout     => 0,
    }
    python::pip { 'pip-mavproxy-sitl':
        pkgname     => 'MAVProxy',
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
    
    # Punch some holes in the firewall for sitl, protect 5770 which mavproxy-sitl uses
    if defined(Class["::maverick_security"]) {
        maverick_security::firewall::firerule { "dev-sitl":
            ports       => [5770-5775],
            ips         => hiera("all_ips"),
            proto       => "tcp"
        }
    }
    
    # Punch some holes in the firewall for sitl mavproxy
    if defined(Class["::maverick_security"]) {
        maverick_security::firewall::firerule { "mavproxy-sitl":
            ports       => [14562-14563],
            ips         => hiera("all_ips"),
            proto       => "udp"
        }
    }
    
    if getvar("maverick_dev::ardupilot::ardupilot_buildsystem") == "make" {
        file { "/etc/systemd/system/dev-sitl.service":
            content     => template("maverick_dev/dev-sitl.service.erb"),
            owner       => "root",
            group       => "root",
            mode        => 644,
            notify      => [ Exec["maverick-systemctl-daemon-reload"], Service["dev-sitl"] ]
        } ->
        service { "dev-sitl":
            ensure      => $sitl_state,
            enable      => true,
            require     => [ Python::Pip['pip-mavproxy-sitl'], Exec["maverick-systemctl-daemon-reload"] ],
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
        file { "/srv/maverick/var/log/dev-sitl":
            ensure      => directory,
            owner       => "mav",
            group       => "mav",
            mode        => 755,
        } ->
        file { "/srv/maverick/data/config/dev-sitl.conf":
            ensure      => present,
            owner       => "mav",
            group       => "mav",
            replace     => false, # initialize but don't overwrite in the future
            source      => "puppet:///modules/maverick_dev/dev-sitl.conf",
            notify      => Service["dev-sitl"]
        } ->
        file { "/srv/maverick/data/config/dev-sitl.screen.conf":
            ensure      => present,
            owner       => "mav",
            group       => "mav",
            source      => "puppet:///modules/maverick_dev/dev-sitl.sim_vehicle.screen.conf",
            notify      => Service["dev-sitl"]
        } ->
        file { "/srv/maverick/software/maverick/bin/dev-sitl.sh":
            ensure      => link,
            target      => "/srv/maverick/software/maverick/manifests/maverick-modules/maverick_dev/files/dev-sitl.sim_vehicle.sh",
            notify      => Service["dev-sitl"]
        } ->
        file { "/etc/systemd/system/dev-sitl.service":
            content     => template("maverick_dev/dev-sitl.sim_vehicle.service.erb"),
            owner       => "root",
            group       => "root",
            mode        => 644,
            notify      => [ Exec["maverick-systemctl-daemon-reload"], Service["dev-sitl"] ]
        } ->
        service { "dev-sitl":
            ensure      => $sitl_state,
            enable      => true,
            require     => [ Python::Pip['pip-mavproxy-sitl'], Exec["maverick-systemctl-daemon-reload"] ],
        }
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
        require       => [ Exec["maverick-systemctl-daemon-reload"], Service["dev-sitl"] ],
    }

}