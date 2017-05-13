class maverick_hardware (
    $sensors = false
) {
	
	# Setup hardware sensors (lmsensors)
	if ($sensors) {
    	class { "maverick_hardware::sensors": }
	}

	if ($raspberry_present == "yes") {
		class { "maverick_hardware::raspberry": }
	}
	
	if ($beagle_present == "yes") {
		class { "maverick_hardware::beagle": }
	}
	
	if ($odroid_present == "yes") {
		class { "maverick_hardware::odroid": }
	}
	
	if ($joule_present == "yes") {
		class { "maverick_hardware::joule": }
	}
	
	# Setup ocam software
	if ($camera_ocam == "yes") {
		class { "maverick_hardware::peripheral::ocam": }
	}
	
	# Setup raspberry pi camera software
	if ($camera_picam == "yes") {
		class { "maverick_hardware::peripheral::picam": }
	}
	
	# Setup realsense depth cameras
	if ($camera_realsense == "yes") {
		class { "maverick_hardware::peripheral::realsense": }
	}
	
}
