class maverick-vision::fpv::init (
    $type = "infrastructure", # 'infrastructure' for normal wifi, 'ap' to act as an Access Point, 'broadcast' for wifibroadcast, MUST match maverick-network::wireless::type
) {

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