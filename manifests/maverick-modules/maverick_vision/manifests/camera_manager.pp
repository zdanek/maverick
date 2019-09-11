class maverick_vision::camera_manager (
    $camera_manager_source = "https://github.com/Dronecode/camera-manager.git",
    $rtsp_port = 8554,
    $active = false,
) {

    # Ensure gstreamer resources are applied before this class
    require maverick_vision::gstreamer
    
    if ! ("install_flag_camera-manager" in $installflags) {
        ensure_packages(["libavahi-common-dev", "libavahi-core-dev", "libavahi-glib-dev", "libavahi-client-dev"])
        oncevcsrepo { "git-camera-manager":
            gitsource   => $camera_manager_source,
            dest        => "/srv/maverick/var/build/camera-manager",
            submodules  => true,
        } ->
        exec { "camera-manager-build":
            user        => "mav",
            timeout     => 0,
            environment => ["PKG_CONFIG_PATH=/srv/maverick/software/gstreamer/lib/pkgconfig:/usr/lib/arm-linux-gnueabihf/pkgconfig"],
            command     => "/srv/maverick/var/build/camera-manager/autogen.sh && CFLAGS='-g -O2' /srv/maverick/var/build/camera-manager/configure --enable-avahi --enable-mavlink --disable-systemd --prefix=/srv/maverick/software/camera-manager && /usr/bin/make -j${::processorcount} && make install >/srv/maverick/var/log/build/camera-manager.build.out 2>&1",
            cwd         => "/srv/maverick/var/build/camera-manager",
            creates     => "/srv/maverick/software/camera-manager/bin/dcm",
            require     => [ Package["libavahi-common-dev"], Class["maverick_vision::gstreamer"] ],
        } ->
        file { "/srv/maverick/var/build/.install_flag_camera-manager":
            ensure      => file,
            owner       => "mav",
        } ->
        file { "/srv/maverick/config/vision/camera-manager.conf":
            source      => "puppet:///modules/maverick_vision/camera-manager.conf",
            owner       => "mav",
            group       => "mav",
            mode        => "644",
            replace     => false,
        }
    }

    file { "/etc/systemd/system/maverick-camera-manager.service":
        source      => "puppet:///modules/maverick_vision/camera-manager.service",
        owner       => "root",
        group       => "root",
        mode        => "644",
        notify      => Exec["maverick-systemctl-daemon-reload"],
    }

    if $active == true {
        service { "maverick-camera-manager":
            ensure      => running,
            enable      => true,
            require     => File["/etc/systemd/system/maverick-camera-manager.service"],
        }
        # Punch some holes in the firewall for rtsp
        if defined(Class["::maverick_security"]) {
            maverick_security::firewall::firerule { "vision-rtsp-udp":
                ports       => [$rtsp_port],
                ips         => lookup("firewall_ips"),
                proto       => "udp", # allow both tcp and udp for rtsp and rtp
            }
            maverick_security::firewall::firerule { "vision-rtsp-tcp":
                ports       => [$rtsp_port],
                ips         => lookup("firewall_ips"),
                proto       => "tcp", # allow both tcp and udp for rtsp and rtp
            }
        }
    } else {
        service { "maverick-camera-manager":
            ensure      => stopped,
            enable      => false,
            require     => File["/etc/systemd/system/maverick-camera-manager.service"],
        }
    }

}
