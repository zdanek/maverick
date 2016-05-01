class maverick-baremetal (
    $sensors = false
) {
	
	# Setup hardware sensors (lmsensors)
	if ($sensors) {
    	include maverick-baremetal::sensors
	}

	if ($raspberry_present == "yes") {
		class { "maverick-baremetal::raspberry::init": }
	}
	
	if ($beagle_present == "yes") {
		class { "maverick-baremetal::beagle::init": }
	}
	
	if ($odroid_present == "yes") {
		class { "maverick-baremetal::odroid::init": }
	}
	
	# Setup ocam software
	if ($camera_ocam == "yes") {
		class { "maverick-baremetal::peripheral::ocam": }
	}
	
	# Setup raspberry pi camera software
	if ($camera_picam == "yes") {
		class { "maverick-baremetal::peripheral::picam": }
	}
	
}
