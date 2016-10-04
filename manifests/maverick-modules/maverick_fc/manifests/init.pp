class maverick_fc (
    $fc_dronekit_source = "http://github.com/dronekit/dronekit-python.git",
    $mavproxy_state = undef,
    $dronekit_la_revision = "master",
) {

    # Install dronekit-la (log analyzer)
    oncevcsrepo { "git-dronekit-la":
        gitsource   => "https://github.com/dronekit/dronekit-la.git",
        dest        => "/srv/maverick/software/dronekit-la",
        revision	=> "${dronekit_la_revision}",
        submodules  => true,
    }
    # Compile dronekit-la
    exec { "compile-dronekit-la":
        command     => "/usr/bin/make -j${::processorcount} >/srv/maverick/var/log/build/dronekit-la.build.log 2>&1",
        creates     => "/srv/maverick/software/dronekit-la/dronekit-la",
        user        => "mav",
        cwd         => "/srv/maverick/software/dronekit-la",
        timeout     => 0,
    }
    
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
    
    python::pip { 'pip-dronekit-fc':
        pkgname     => 'dronekit',
        virtualenv  => '/srv/maverick/.virtualenvs/fc',
        ensure      => present,
        owner       => 'mav',
        timeout     => 0,
    }
    python::pip { 'pip-mavproxy-fc':
        pkgname     => 'MAVProxy',
        virtualenv  => '/srv/maverick/.virtualenvs/fc',
        ensure      => present,
        owner       => 'mav',
        timeout     => 0,
    }
        
    file { "/srv/maverick/var/log/mavproxy-fc":
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
        source      => "puppet:///modules/maverick_fc/mavproxy-fc.conf",
        notify      => Service["mavproxy-fc"]
    } ->
    file { "/srv/maverick/data/config/mavproxy-fc.screen.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        source      => "puppet:///modules/maverick_fc/mavproxy-fc.screen.conf",
        notify      => Service["mavproxy-fc"]
    } ->
    file { "/srv/maverick/software/maverick/bin/mavproxy-fc.sh":
        ensure      => link,
        target      => "/srv/maverick/software/maverick/manifests/maverick-modules/maverick_fc/files/mavproxy-fc.sh",
    } ->
    file { "/etc/systemd/system/mavproxy-fc.service":
        content     => template("maverick_fc/mavproxy-fc.service.erb"),
        owner       => "root",
        group       => "root",
        mode        => 644,
        notify      => [ Exec["maverick-systemctl-daemon-reload"], Service["mavproxy-fc"] ]
    } ->
    service { "mavproxy-fc":
        ensure      => $mavproxy_state,
        enable      => true,
        require       => Exec["maverick-systemctl-daemon-reload"]
    }
    
    # Punch some holes in the firewall for mavproxy
    if defined(Class["::maverick_security"]) {
        maverick_security::firewall::firerule { "mavproxy-fc":
            ports       => [14550-14555],
            ips         => hiera("all_ips"),
            proto       => "udp"
        }
    }
   
        
}