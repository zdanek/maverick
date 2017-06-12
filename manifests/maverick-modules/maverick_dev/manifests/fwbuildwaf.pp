# Define function to build ardupilot firmwares using new waf system
define maverick_dev::fwbuildwaf ($build, $board) {
    if ! ("${board}/bin/ardu${build}" in $waffiles or "${board}/bin/${build}" in $waffiles) {
        warning("Ardupilot Firmware: ${build} will be compiled and can take a while, please be patient")
        exec { "ardupilotfw_${board}_${build}":
            user        => "mav",
            timeout     => 0,
            command     => "/srv/maverick/code/ardupilot/waf configure --board ${board} && /srv/maverick/code/ardupilot/waf ${build} >/srv/maverick/var/log/build/ardupilot-fw-${build}.build.log 2>&1",
            cwd         => "/srv/maverick/code/ardupilot",
            creates     => "/srv/maverick/code/ardupilot/${build}.elf",
            require     => Install_python_module['pip-future']
        }
    }
}
