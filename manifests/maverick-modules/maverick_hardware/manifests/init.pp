# @summary
#   Maverick_hardware class
#   This class controls all other classes in maverick_hardware module.
#
# @example Declaring the class
#   This class is included from the base manifest and is not usually included elsewhere.
#
# @param raspberry_install
#   If true, include the maverick_hardware::raspberry support class
#	@param beagle_install
#		If true, include the maverick_hardware::beagle support class
# @param odroid_install
#		If true, include the maverick_hardware::odroid support class
# @param joule_install
#		If true, include the maverick_hardware::joule support class
# @param up_install
#		If true, include the maverick_hardware::up support class
# @param tegra_install
#		If true, include the maverick_hardware::tegra support class
# @param camera_ocam_install
#		If true, include the maverick_hardware::camera_ocam support class
# @param camera_picam_install
#		If true, include the maverick_hardware::camera_picam support class
# @param realsense install
#		If true, include the maverick_hardware::realsense support class
# @param flirone_install
#		If true, include the maverick_hardware::flirone support class
#
class maverick_hardware (
    Boolean $raspberry_install = false,
    Boolean $beagle_install = false,
    Boolean $odroid_install = false,
    Boolean $joule_install = false,
    Boolean $up_install = false,
    Boolean $tegra_install = false,
    Boolean $camera_ocam_install = false,
    Boolean $camera_picam_install = false,
    Boolean $realsense_install = true,
    Boolean $flirone_install = false,
) {
	
	# Create status.d directory for maverick status`
	file { "/srv/maverick/software/maverick/bin/status.d/124.hardware":
			ensure      => directory,
			owner       => "mav",
			group       => "mav",
			mode        => "755",
	} ->
	file { "/srv/maverick/software/maverick/bin/status.d/124.hardware/__init__":
			owner       => "mav",
			content     => "Hardware Services",
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
