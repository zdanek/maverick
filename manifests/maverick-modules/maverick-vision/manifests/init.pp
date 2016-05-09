class maverick-vision (
    $mjpg_streamer = false,
    $fpv = true,
    $cv = false,
) {
    
    # Setup standard packages for all platforms
    ensure_packages(["gstreamer1.0-plugins-base", "gstreamer1.0-plugins-good", "gstreamer1.0-plugins-bad", "gstreamer1.0-plugins-ugly", "gstreamer1.0-tools", "python-gst-1.0", "gir1.2-gstreamer-1.0", "gir1.2-gst-plugins-base-1.0", "gir1.2-clutter-gst-2.0"])
    ensure_packages(["v4l-utils", "v4l-conf","uvcdynctrl"])
    ensure_packages(["x264"])

    if $mjpg_streamer == true  {
        class { "maverick-vision::fpv::mjpg-streamer": }
    }
    
    if $fpv == true {
        class { "maverick-vision::fpv::init": }
    }
    
    if $cv == true {
        class { "maverick-vision::cv::init": }
    }
    
}