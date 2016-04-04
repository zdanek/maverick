class maverick-dronekit::fc (
    $fc_mavproxy_master = "/dev/ttyACM0", # FlightController -> CompanionComputer connection
    $fc_dronekit_source = "http://github.com/dronekit/dronekit-python.git",
) {
    
    # Install a virtual environment for dronekit fc
    file { "/srv/maverick/code/dronekit-fc":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => 755,
    } ->
    python::virtualenv { '/srv/maverick/code/dronekit-fc':
        ensure       => present,
        version      => 'system',
        systempkgs   => false,
        distribute   => true,
        venv_dir     => '/srv/maverick/.virtualenvs/dronekit-fc',
        owner        => 'mav',
        group        => 'mav',
        cwd          => '/srv/maverick/code/dronekit-fc',
        timeout      => 0,
    } ->
    vcsrepo { "/srv/maverick/code/dronekit-fc/dronekit-python":
        ensure		=> present,
        provider 	=> git,
        source		=> $fc_dronekit_source,
        revision	=> "master",
        owner		=> "mav",
        group		=> "mav",
    }
    
    python::pip { 'pip-mavproxy-fc':
        pkgname     => 'MAVProxy',
        virtualenv  => '/srv/maverick/.virtualenvs/dronekit-fc',
        ensure      => present,
        owner       => 'mav',
        timeout     => 0,
    }
        
    file { "/etc/systemd/system/fc-mavproxy.service":
        content     => template("maverick-dronekit/fc-mavproxy.service.erb"),
        owner       => "root",
        group       => "root",
        mode        => 644,
        notify      => Exec["maverick-systemctl-daemon-reload"],
    } ->
    service { "fc-mavproxy":
        ensure      => running,
        enable      => true,
    }
    # Punch some holes in the firewall for sitl
    if defined(Class["::maverick-security"]) {
        maverick-security::firewall::firerule { "fc-mavproxy":
            ports       => [14550-14555],
            ips         => hiera("all_ips"),
            proto       => "udp"
        }
    }
    
}