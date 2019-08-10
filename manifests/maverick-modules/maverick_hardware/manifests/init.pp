class maverick_hardware (
    $sensors = false,
    $raspberry_install = false,
    $beagle_install = false,
    $odroid_install = false,
    $joule_install = false,
    $up_install = false,
    $tegra_install = false,
    $camera_ocam_install = false,
    $camera_picam_install = false,
    $realsense_install = true,
    $flirone_install = true,
) {
	
	# Setup hardware sensors (lmsensors)
	if $sensors {
    	class { "maverick_hardware::sensors": }
	}

	if $raspberry_present == "yes" or $raspberry_install == true {
		class { "maverick_hardware::raspberry": }
	}
	
	if $beagle_present == "yes" or $beagle_install == true {
		class { "maverick_hardware::beagle": }
	}
	
	if $odroid_present == "yes" or $odroid_install == true {
		class { "maverick_hardware::odroid": }
	}
	
	if $joule_present == "yes" or $joule_install == true {
		class { "maverick_hardware::joule": }
	}
	
	if $up_present == "yes" or $up_install == true {
		class { "maverick_hardware::up": }
	}
	
	if $tegra_present == "yes" or $tegra_install == true {
		class { "maverick_hardware::tegra": }
	}

	# Setup ocam software
	if $camera_ocam == "yes" or $camera_ocam_install == true {
		class { "maverick_hardware::peripheral::ocam": }
	}
	
	# Setup raspberry pi camera software
	if $camera_picam == "yes" or $camera_picam_install == true {
		class { "maverick_hardware::peripheral::picam": }
	}
	
	# Setup realsense depth cameras
	if $camera_realsense == "yes" or $realsense_install == true {
		class { "maverick_hardware::peripheral::realsense": }
	}
	
	# Setup FlirOne thermal cameras
	if $flirone_present == "yes" or $flirone_install == true {
		class { "maverick_hardware::peripheral::flirone": }
	}
	
}
