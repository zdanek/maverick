# @summary
#   Maverick_vision::Visiond class
#   This class installs and manages the Maverick visiond software.
#
# @example Declaring the class
#   This class is included from maverick_vision class and should not be included from elsewhere
#
# @param active
#   If true, start the visiond service and enable at boot time.
# @param rtsp_port
#   Port number to listen on for RTSP stream requests - 5600 is the 'default' RTSP port
# @param webvision_active
#   If true, start the webvision service and enable at boot time.
#
class maverick_vision::visiond (
    Boolean $active = true,
    Integer $rtsp_port = 5600,
    Boolean $webvision_active = true,
) {

    # Setup standard packages for all platforms
    ensure_packages(["v4l-utils", "v4l-conf","uvcdynctrl"])

    # Add v4l2 python bindings
    install_python_module { "pip-v4l2":
        pkgname     => "v4l2",
        url         => "git+https://github.com/fnoop/python-v4l2.git",
        ensure      => present,
        timeout     => 0,
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
        service { "maverick-visiond":
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
        service { "maverick-visiond":
            ensure      => stopped,
            enable      => false,
            require     => Class["maverick_vision::gstreamer"],
        }
    }
    
    if defined(Class["::maverick_web"]) {
        # Add a temporary service to run web vision
        file { "/etc/systemd/system/maverick-webvision.service":
            ensure          => present,
            mode            => "644",
            owner           => "mav",
            group           => "mav",
            source          => "puppet:///modules/maverick_vision/maverick-webvision.service",
            notify          => Exec["maverick-systemctl-daemon-reload"],
        } ->
        nginx::resource::location { "web-webvision":
            location    => "/vision/webvision/",
            proxy       => 'http://localhost:6793/',
            server      => getvar("maverick_web::server_fqdn"),
            require     => [ Class["maverick_web::fcs"], Class["nginx"], ],
        }

        if $webvision_active == true {
            service { "maverick-webvision":
                ensure          => running,
                enable          => true,
                require         => Exec["maverick-systemctl-daemon-reload"],
            }
        } else {
            service { "maverick-webvision":
                ensure          => stopped,
                enable          => false,
                require         => Exec["maverick-systemctl-daemon-reload"],
            }
        }
    }
    
    # status.d entry
    file { "/srv/maverick/software/maverick/bin/status.d/123.vision/101.visiond.status":
        owner   => "mav",
        content => "visiond,Vision Service\n",
    }
    file { "/srv/maverick/software/maverick/bin/status.d/123.vision/107.webvision.status":
        owner   => "mav",
        content => "webvision,Web Vision\n",
    }
}
