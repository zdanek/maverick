# Define function to build ardupilot firmwares using new waf system
define maverick_dev::fwbuildwaf (
    $buildfile,
    $vehicle,
    $board
) {
    if ! ("${board}/bin/${buildfile}" in $waffiles) {
        warning("Ardupilot Firmware: ${vehicle} will be compiled and can take a while, please be patient")
        exec { "ardupilotfw_${board}_${vehicle}":
            user        => "mav",
            timeout     => 0,
            command     => "/srv/maverick/code/ardupilot/waf configure --board ${board} && /srv/maverick/code/ardupilot/waf ${vehicle} >/srv/maverick/var/log/build/ardupilot-fw-${vehicle}.build.log 2>&1",
            cwd         => "/srv/maverick/code/ardupilot",
            creates     => "/srv/maverick/code/ardupilot/build/${board}/bin/${buildfile}",
            require     => Install_python_module['pip-future']
        }
    }
}
