class maverick_vision::visiond (
    $active = true,
    $rtsp_port = 5600,
    $webvision = true,
) {

    # Setup standard packages for all platforms
    ensure_packages(["v4l-utils", "v4l-conf","uvcdynctrl"])

    # Add v4l2 python bindings
    install_python_module { 'pip-v4l2':
        pkgname     => 'v4l2',
        ensure      => present,
    }
     
    # Link maverick-visiond into central bin directory
    file { "/srv/maverick/software/maverick/bin/maverick-visiond":
        ensure      => link,
        target      => "/srv/maverick/software/maverick/manifests/maverick-modules/maverick_vision/files/maverick-visiond",
    }
    file { "/srv/maverick/config/vision/maverick-visiond.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        replace     => false, # initialize but don't overwrite in the future
        source      => "puppet:///modules/maverick_vision/maverick-visiond.conf",
    }
    
    # Add visiond as a service
    file { "/etc/systemd/system/maverick-visiond.service":
        source      => "puppet:///modules/maverick_vision/maverick-visiond.service",
        owner       => "root",
        group       => "root",
        mode        => "644",
        notify      => Exec["maverick-systemctl-daemon-reload"],
    } ->
    file { "/etc/systemd/system/maverick-visiond.service.d":
        ensure      => directory
    } ->
    file { "/etc/systemd/system/maverick-visiond.service.d/typelib-path.conf":
        ensure      => present,
        mode        => "644",
        content     => "[Service]\nEnvironment=\"GI_TYPELIB_PATH=/srv/maverick/software/gstreamer/lib/girepository-1.0:/usr/lib/girepository-1.0\""
    } ->
    file { "/etc/systemd/system/maverick-visiond.service.d/library-path.conf":
        ensure      => present,
        mode        => "644",
        content     => "[Service]\nEnvironment=\"LD_LIBRARY_PATH=/srv/maverick/software/gstreamer/lib\""
    } ->
    file { "/etc/systemd/system/maverick-visiond.service.d/path.conf":
        ensure      => present,
        mode        => "644",
        content     => "[Service]\nEnvironment=\"PATH=/srv/maverick/software/gstreamer/bin:/usr/sbin:/usr/bin:/sbin:/bin\""
    }
    
    if $active == true {
        service_wrapper { "maverick-visiond":
            ensure      => running,
            enable      => true,
            require     => Class["maverick_vision::gstreamer"],
        }
        # Punch some holes in the firewall for rtsp
        if defined(Class["::maverick_security"]) {
            maverick_security::firewall::firerule { "visiond-rtsp-udp":
                ports       => [$rtsp_port],
                ips         => lookup("firewall_ips"),
                proto       => "udp", # allow both tcp and udp for rtsp and rtp
            }
            maverick_security::firewall::firerule { "visiond-rtsp-tcp":
                ports       => [$rtsp_port],
                ips         => lookup("firewall_ips"),
                proto       => "tcp", # allow both tcp and udp for rtsp and rtp
            }
        }
    } else {
        service_wrapper { "maverick-visiond":
            ensure      => stopped,
            enable      => false,
            require     => Class["maverick_vision::gstreamer"],
        }
    }
    
    if $webvision == true {
        if defined(Class["::maverick_web"]) {
            # Add a temporary service to run web vision
            install_python_module { "webvision-tornado":
                pkgname     => "tornado",
                ensure      => atleast,
                version     => "4.5.2",
            } ->
            file { "/etc/systemd/system/maverick-webvision.service":
                ensure          => present,
                mode            => "644",
                owner           => "mav",
                group           => "mav",
                source          => "puppet:///modules/maverick_vision/maverick-webvision.service",
                notify          => Exec["maverick-systemctl-daemon-reload"],
            } ->
            service_wrapper{ "maverick-webvision":
                ensure          => running,
                enable          => true,
            } ->
            nginx::resource::location { "web-webvision":
                location    => "/vision/webvision/",
                proxy       => 'http://localhost:6793/',
                server      => "${::hostname}.local",
                require     => [ Class["maverick_gcs::fcs"], Class["nginx"], Service_wrapper["maverick-webvision"], ],
            }
        }
    }

}