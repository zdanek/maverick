# @summary
#   Define function to build ardupilot firmware using new waf system
#
# @example
#   @@maverick_dev::fwbuildwaf { 'sitl_waf_copter':
#       board       => "sitl",
#       vehicle     => "copter",
#       buildfile   => "arducopter",
#   }
#
# @note
#   Can be called many times to compile different vhicles and target hardware
#
# @param buildfile
#   Filename of the compiled firwmare.  This is used to test if the compiled firmware already exists.
# @param vehicle
#   The vehicle type to compile, eg. copter, plane, rover, sub, heli, antennatracker
# @param board
#   Hardware target - eg. sitl, px4-v4
#
define maverick_dev::fwbuildwaf (
    String $buildfile,
    String $vehicle,
    String $board
) {
    if ! ("${board}/bin/${buildfile}" in $waffiles) {
        warning("Ardupilot Firmware: ${vehicle} will be compiled and can take a while, please be patient")
        exec { "ardupilotfw_${board}_${vehicle}":
            user        => "mav",
            timeout     => 0,
            environment => ["PATH=/srv/maverick/software/python/bin:/usr/bin:/bin", "PYTHON=/srv/maverick/software/python/bin/python3", "PYTHONPATH=/srv/maverick/software/python/lib/python3.7/site-packages", "PKG_CONFIG_PATH=/usr/lib/arm-linux-gnueabihf/pkgconfig"],
            command     => "/srv/maverick/software/ardupilot/waf configure --python=/srv/maverick/software/python/bin/python3 --board ${board} && /srv/maverick/software/ardupilot/waf ${vehicle} >/srv/maverick/var/log/build/ardupilot-fw-${vehicle}.build.log 2>&1",
            cwd         => "/srv/maverick/software/ardupilot",
            creates     => "/srv/maverick/software/ardupilot/build/${board}/bin/${buildfile}",
            require     => [ Install_python_module['pip-future'], Install_python_module['pip-empy'] ],
        }
    }
}
