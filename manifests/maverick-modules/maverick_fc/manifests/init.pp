class maverick_fc (
    $fc_dronekit_source = "http://github.com/dronekit/dronekit-python.git",
    $mavlink_proxy = "mavproxy",
    $mavlink_active = true,
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
        
    file { "/srv/maverick/var/log/mavlink-fc":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => 755,
    }
   
    ### Setup mavlink proxy
    if $mavlink_proxy == "mavproxy" {
        maverick_mavlink::cmavnode { "fc":
            active      => false
        } ->
        maverick_mavlink::mavlinkrouter { "fc":
            active      => false
        } ->
        maverick_mavlink::mavproxy { "fc":
            input       => "/dev/ttyAMA0",
            instance    => 2,
            startingudp => 14570,
            startingtcp => 5780,
            active      => $mavlink_active,
        }
    } elsif $mavlink_proxy == "cmavnode" {
        maverick_mavlink::mavproxy { "fc":
            active      => false
        } ->
        maverick_mavlink::mavlinkrouter { "fc":
            active      => false
        } ->
        maverick_mavlink::cmavnode { "fc":
            input       => "/dev/ttyAMA0",
            startingudp => 14570,
            startingtcp => 5780,
            active      => $mavlink_active,
        }
    } elsif $mavlink_proxy == "mavlinkrouter" {
        maverick_mavlink::cmavnode { "fc":
            active      => false
        } ->
        maverick_mavlink::mavproxy { "fc":
            active      => false
        } ->
        maverick_mavlink::mavlinkrouter { "fc":
            input       => "/dev/ttyAMA0",
            startingudp => 14570,
            startingtcp => 5780,
            active      => $mavlink_active,
        }
    }
}