class maverick_vision::visiond (
    $visiond_state = undef,
) {
        
    # Setup standard packages for all platforms
    ensure_packages(["v4l-utils", "v4l-conf","uvcdynctrl"])
    ensure_packages(["x264"])

    # Add v4l2 python bindings
    python::pip { 'pip-v4l2':
        pkgname     => 'v4l2',
        ensure      => present,
    }
     
    # Link maverick-visiond into central bin directory
    file { "/srv/maverick/software/maverick/bin/maverick-visiond":
        ensure      => link,
        target      => "/srv/maverick/software/maverick/manifests/maverick-modules/maverick_vision/files/maverick-visiond",
    }
    file { "/srv/maverick/data/config/maverick-visiond.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        replace     => false, # initialize but don't overwrite in the future
        source      => "puppet:///modules/maverick_vision/maverick-visiond.conf",
    }
    
    # Add visiond as a service
    file { "/etc/systemd/system/maverick-visiond.service":
        content     => template("maverick_vision/maverick-visiond.service.erb"),
        owner       => "root",
        group       => "root",
        mode        => 644,
        notify      => Exec["maverick-systemctl-daemon-reload"],
    } ->
    service { "maverick-visiond":
        ensure      => $visiond_state,
        enable      => true,
    }
    
}