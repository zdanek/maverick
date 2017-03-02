class maverick_baremetal (
    $sensors = false
) {
	
	# Setup hardware sensors (lmsensors)
	if ($sensors) {
    	class { "maverick_baremetal::sensors": }
	}

	if ($raspberry_present == "yes") {
		class { "maverick_baremetal::raspberry": }
	}
	
	if ($beagle_present == "yes") {
		class { "maverick_baremetal::beagle": }
	}
	
	if ($odroid_present == "yes") {
		class { "maverick_baremetal::odroid": }
	}
	
	if ($joule_present == "yes") {
		class { "maverick_baremetal::joule": }
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
