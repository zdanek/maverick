class maverick_baremetal (
    $sensors = false
) {
	
	# Setup hardware sensors (lmsensors)
	if ($sensors) {
    	include maverick_baremetal::sensors
	}

	if ($raspberry_present == "yes") {
		class { "maverick_baremetal::raspberry::init": }
	}
	
	if ($beagle_present == "yes") {
		class { "maverick_baremetal::beagle::init": }
	}
	
	if ($odroid_present == "yes") {
		class { "maverick_baremetal::odroid::init": }
	}
	
	# Setup ocam software
	if ($camera_ocam == "yes") {
		class { "maverick_baremetal::peripheral::ocam": }
	}
	
	# Setup raspberry pi camera software
	if ($camera_picam == "yes") {
		class { "maverick_baremetal::peripheral::picam": }
	}
	
}
