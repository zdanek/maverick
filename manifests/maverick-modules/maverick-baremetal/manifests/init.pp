class maverick-baremetal (
    $sensors = false
) {
	
	### Setup hardware sensors (lmsensors)
	if ($sensors) {
    	include maverick-baremetal::sensors
	}

	if ($raspberry_present == "yes") {
		class { "maverick-baremetal::raspberry::init": }
	}
}
