class maverick-baremetal (
    $sensors = false
) {
	
	### Setup hardware sensors (lmsensors)
	if ($sensors) {
    	include maverick-baremetal::sensors
	}

	if ($raspberry_model) {
		class { "maverick-baremetal::raspberry": }
	}
}