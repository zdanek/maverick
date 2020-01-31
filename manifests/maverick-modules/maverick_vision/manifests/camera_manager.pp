# @summary
#   Maverick_vision::Camera_manager class
#   This class installs and manages the Intel CameraManager software.
#
# @example Declaring the class
#   This class is included from maverick_vision class and should not be included from elsewhere
#
# @param active
#   If true, start the service and enable at boot time.
# @param gitsource
#   Which Git repo to use to compile/install the software.
# @param rtsp_port
#   The port number to listen on for RTSP.
#
class maverick_vision::camera_manager (
    Boolean $active = false,
    String $gitsource = "https://github.com/Dronecode/camera-manager.git",
    Integer $rtsp_port = 8554,
) {

    # Ensure gstreamer resources are applied before this class
    require maverick_vision::gstreamer
    
    if ! ("install_flag_camera-manager" in $installflags) {
        ensure_packages(["libavahi-common-dev", "libavahi-core-dev", "libavahi-glib-dev", "libavahi-client-dev"])
        oncevcsrepo { "git-camera-manager":
            gitsource   => $gitsource,
            dest        => "/srv/maverick/var/build/camera-manager",
            submodules  => true,
        } ->
        exec { "camera-manager-build":
            user        => "mav",
            timeout     => 0,
            environment => ["PKG_CONFIG_PATH=/srv/maverick/software/gstreamer/lib/pkgconfig:/usr/lib/arm-linux-gnueabihf/pkgconfig"],
            command     => "/srv/maverick/var/build/camera-manager/autogen.sh && CFLAGS='-g -O2' /srv/maverick/var/build/camera-manager/configure --enable-avahi --enable-mavlink --disable-systemd --prefix=/srv/maverick/software/camera-manager >/srv/maverick/var/log/build/camera-manager.configure.log 2>&1 && /usr/bin/make -j${::processorcount} && make install >/srv/maverick/var/log/build/camera-manager.build.log 2>&1",
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

    # status.d entry
    file { "/srv/maverick/software/maverick/bin/status.d/123.vision/102.camera-manager.status":
        owner   => "mav",
        content => "camera-manager,Intel Camera Manager\n",
    }

}
