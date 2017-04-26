class maverick_vision::visiond (
    $active = true,
) {

    # Setup standard packages for all platforms
    ensure_packages(["v4l-utils", "v4l-conf","uvcdynctrl"])

    # Add v4l2 python bindings
    install_python_module { 'pip-v4l2':
        pkgname     => 'v4l2',
        ensure      => present,
    }
     
    # Create log directory
    file { "/srv/maverick/var/log/vision":
        owner       => "mav",
        group       => "mav",
        mode        => "755",
        ensure      => "directory",
    }
    
    # Link maverick-visiond into central bin directory
    file { "/srv/maverick/software/maverick/bin/maverick-visiond":
        ensure      => link,
        target      => "/srv/maverick/software/maverick/manifests/maverick-modules/maverick_vision/files/maverick-visiond",
    }
    file { "/srv/maverick/data/config/vision/maverick-visiond.conf":
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
    file { "/etc/systemd/system/maverick-visiond.service.d":
        ensure      => directory
    } ->
    file { "/etc/systemd/system/maverick-visiond.service.d/typelib-path.conf":
        ensure      => present,
        mode        => 644,
        content     => "[Service]\nEnvironment=\"GI_TYPELIB_PATH=/srv/maverick/software/gstreamer/lib/girepository-1.0:/usr/lib/girepository-1.0\""
    } ->
    file { "/etc/systemd/system/maverick-visiond.service.d/library-path.conf":
        ensure      => present,
        mode        => 644,
        content     => "[Service]\nEnvironment=\"LD_LIBRARY_PATH=/srv/maverick/software/gstreamer/lib\""
    } ->
    file { "/etc/systemd/system/maverick-visiond.service.d/path.conf":
        ensure      => present,
        mode        => 644,
        content     => "[Service]\nEnvironment=\"PATH=/srv/maverick/software/gstreamer/bin:/usr/sbin:/usr/bin:/sbin:/bin\""
    }
    
    if $active == true {
        service { "maverick-visiond":
            ensure      => running,
            enable      => true,
            require     => Class["maverick_vision::gstreamer"],
        }
    } else {
        service { "maverick-visiond":
            ensure      => stopped,
            enable      => false,
            require     => Class["maverick_vision::gstreamer"],
        }
    }
    
}