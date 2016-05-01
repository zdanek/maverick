class maverick-vision (
    $mjpg_streamer = false,
) {
    
    # Setup standard packages for all platforms
    ensure_packages(["gstreamer1.0-plugins-base", "gstreamer1.0-plugins-good", "gstreamer1.0-plugins-bad", "gstreamer1.0-plugins-ugly", "gstreamer1.0-tools", "python-gst-1.0"])
    ensure_packages(["v4l-utils", "v4l-conf","uvcdynctrl"])
    ensure_packages(["x264"])

    if $mjpg_streamer == true  {
        class { "maverick-vision::fpv::mjpg-streamer": }
    }
    
}