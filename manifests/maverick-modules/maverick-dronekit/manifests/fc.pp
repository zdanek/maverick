class maverick-dronekit::fc (
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
    file { "/srv/maverick/.virtualenvs/dronekit-fc/lib/python2.7/no-global-site-packages.txt":
        ensure  => absent
    } ->
    oncevcsrepo { "git-fc-dronekit-python":
        gitsource   => $fc_dronekit_source,
        dest        => "/srv/maverick/code/dronekit-fc/dronekit-python",
    }
    
    python::pip { 'pip-mavproxy-fc':
        pkgname     => 'MAVProxy',
        virtualenv  => '/srv/maverick/.virtualenvs/dronekit-fc',
        ensure      => present,
        owner       => 'mav',
        timeout     => 0,
    }
        
    file { "/srv/maverick/data/logs/mavproxy-fc":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => 755,
    }
    
    file { "/srv/maverick/data/config/mavproxy-fc.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        replace     => false, # initialize but don't overwrite in the future
        source      => "puppet:///modules/maverick-dronekit/mavproxy-fc.conf",
    } ->
    file { "/srv/maverick/software/maverick/bin/mavproxy-fc.sh":
        ensure      => link,
        target      => "/srv/maverick/software/maverick/manifests/maverick-modules/maverick-dronekit/files/mavproxy-fc.sh",
    } ->
    file { "/etc/systemd/system/mavproxy-fc.service":
        content     => template("maverick-dronekit/mavproxy-fc.service.erb"),
        owner       => "root",
        group       => "root",
        mode        => 644,
        notify      => [ Exec["maverick-systemctl-daemon-reload"], Service["mavproxy-fc"] ]
    } ->
    service { "mavproxy-fc":
        ensure      => running,
        enable      => true,
        require       => Exec["maverick-systemctl-daemon-reload"]
    }
    
    # Punch some holes in the firewall for mavproxy
    if defined(Class["::maverick-security"]) {
        maverick-security::firewall::firerule { "mavproxy-fc":
            ports       => [14550-14555],
            ips         => hiera("all_ips"),
            proto       => "udp"
        }
    }
    
}