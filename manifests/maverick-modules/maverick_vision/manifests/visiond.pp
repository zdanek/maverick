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
# @param webrtc_port
#   Port number to listen on for WebRTC Signalling and protocol negotiation
#
class maverick_vision::visiond (
    Boolean $active = true,
    Integer $rtsp_port = 6010,
    Integer $webrtc_port = 6011,
) {

    # Setup standard packages for all platforms
    ensure_packages(["v4l-utils", "v4l-conf","uvcdynctrl"])

    # Add v4l2 python bindings
    install_python_module { "pip-v4l2":
        pkgname     => "v4l2",
        url         => "git+https://github.com/fnoop/python-v4l2.git",
        ensure      => present,
        timeout     => 0,
    } ->

    # Add zeroconf python module
    install_python_module { "pip-visiond-zeroconf":
        pkgname     => "zeroconf",
        ensure      => present,
        timeout     => 0,
    } ->
    
    # Add sdnotify python module
    install_python_module { "pip-visiond-sdnotify":
        pkgname     => "sdnotify",
        ensure      => present,
        timeout     => 0,
    }

    # Pull visiond repo
    oncevcsrepo { "git-visiond":
        gitsource   => "https://github.com/goodrobots/visiond.git",
        dest        => "/srv/maverick/software/visiond",
    } ->
    
    file { "/srv/maverick/config/vision/visiond.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        replace     => false, # initialize but don't overwrite in the future
        content     => template("maverick_vision/visiond.conf.erb"),
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
    file { "/etc/systemd/system/maverick-visiond.service.d/openssl.conf":
        ensure      => present,
        mode        => "644",
        content     => "[Service]\nEnvironment=\"OPENSSL_CONF=\"",
    }
    
    if $active == true {
        service { "maverick-visiond":
            ensure      => running,
            enable      => true,
            require     => [ Class["maverick_vision::gstreamer"], Install_python_module["pip-visiond-sdnotify"] ],
        }
        # Punch some holes in the firewall for rtsp
        if defined(Class["::maverick_security"]) {
            maverick_security::firewall::firerule { "visiond-udp":
                ports       => [$rtsp_port, $webrtc_port],
                ips         => lookup("firewall_ips"),
                proto       => "udp", # allow both tcp and udp for rtsp and rtp
            }
            maverick_security::firewall::firerule { "visiond-tcp":
                ports       => [$rtsp_port, $webrtc_port],
                ips         => lookup("firewall_ips"),
                proto       => "tcp", # allow both tcp and udp for rtsp and rtp
            }
        }
    } else {
        service { "maverick-visiond":
            ensure      => stopped,
            enable      => false,
            require     => [ Class["maverick_vision::gstreamer"], Install_python_module["pip-visiond-sdnotify"] ],
        }
    }
    
    # status.d entry
    file { "/srv/maverick/software/maverick/bin/status.d/123.vision/101.visiond.status":
        owner   => "mav",
        content => "visiond,Vision Service\n",
    }

    # Remove legacy webvision service
    service { "maverick-webvision":
        ensure          => stopped,
        enable          => false,
        require         => Exec["maverick-systemctl-daemon-reload"],
    } ->
    file { "/etc/systemd/system/maverick-webvision.service":
        ensure          => absent,
    } ->
    file { "/srv/maverick/software/maverick/bin/status.d/123.vision/107.webvision.status":
        ensure          => absent,
    }

}
