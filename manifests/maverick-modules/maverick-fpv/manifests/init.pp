class maverick-fpv (
    $type = "infrastructure", # 'infrastructure' for normal wifi, 'ap' to act as an Access Point, 'broadcast' for wifibroadcast, MUST match maverick-network::wireless::type
) {
    
    # Setup standard packages for all platforms
    ensure_packages(["gstreamer1.0-plugins-base", "gstreamer1.0-plugins-good", "gstreamer1.0-plugins-bad", "gstreamer1.0-plugins-ugly", "gstreamer1.0-tools", "python-gst-1.0"])
    ensure_packages(["v4l-utils", "v4l-conf","uvcdynctrl"])
    
    # Setup software necessary for fpv.  Some of the packages are platform specific
    if ($raspberry_present == "yes") {
		# ensure_packages(["omxplayer"]) # lots of dependencies, don't use until we have to
		ensure_packages(["gstreamer1.0-omx"])
	} elsif ($beagle_present == "yes") {
		
	} elsif ($odroid_present == "yes") {
		
	}
    
    if $type == "infrastructure" {
        #class { "maverick-fpv::infrastructure": }
    } elsif $type == "ap" {
        class { "maverick-fpv::ap": }
    } elsif $type == "broadcast" {
        class { "maverick-fpv::broadcast": }
    }
    
}