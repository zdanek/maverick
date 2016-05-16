class maverick-vision (
    $fpv = true,
    $cv = false,
    $gstreamer_installtype = "native",
    $mjpg_streamer = false,
) {
    
    # Setup standard packages for all platforms
    ensure_packages(["v4l-utils", "v4l-conf","uvcdynctrl"])
    ensure_packages(["x264"])

    # Install gstreamer
    if $gstreamer_installtype == "native" {
        ensure_packages(["gstreamer1.0-plugins-base", "gstreamer1.0-plugins-good", "gstreamer1.0-plugins-bad", "gstreamer1.0-plugins-ugly", "gstreamer1.0-tools", "python-gst-1.0", "gir1.2-gstreamer-1.0", "gir1.2-gst-plugins-base-1.0", "gir1.2-clutter-gst-2.0"])
    } else {
        package { ["libgstreamer1.0-0"]:
            ensure      => asbent
        }
    }

    if $mjpg_streamer == true  {
        class { "maverick-vision::fpv::mjpg-streamer": }
    }
    
    if $fpv == true {
        class { "maverick-vision::fpv::init": }
    }
    
    if $cv == true {
        class { "maverick-vision::cv::init": }
    }
    
    # Link maverick-visiond into central bin directory
    file { "/srv/maverick/software/maverick/bin/maverick-visiond":
        ensure      => link,
        target      => "/srv/maverick/software/maverick/manifests/maverick-modules/maverick-vision/files/maverick-visiond",
    }
    
    # Add visiond as a service
    file { "/etc/systemd/system/maverick-visiond.service":
        content     => template("maverick-vision/maverick-visiond.service.erb"),
        owner       => "root",
        group       => "root",
        mode        => 644,
        notify      => Exec["maverick-systemctl-daemon-reload"],
    } ->
    service { "maverick-visiond":
        ensure      => running,
        enable      => true
    }
    
}