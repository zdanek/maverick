class maverick_dev::ardupilot (
    $ardupilot_source = "https://github.com/ArduPilot/ardupilot.git",
    $ardupilot_branch = "master", # eg. master, Copter-3.3, ArduPlane-release
    $ardupilot_type = "plane", # copter/plane/rover
    $ardupilot_board = "px4-v2",
    $ardupilot_buildsystem = "waf", # waf (Copter >=3.4) or make (Copter <3.3)
    $sitl,
) {

    # Install ardupilot from git
    oncevcsrepo { "git-ardupilot":
        gitsource   => $ardupilot_source,
        dest        => "/srv/maverick/code/ardupilot",
        revision	=> $ardupilot_branch,
    }
    ensure_packages(["make", "gawk", "g++", "arduino-core", "gcc-arm-none-eabi", "binutils-arm-none-eabi", "gdb-arm-none-eabi", "genromfs", "python-empy"])
    
    # Define function to build ardupilot firmwares using new waf system
    define fwbuildwaf ($build, $board) {
        if ! ("${board}/bin/ardu${build}" in $waffiles) {
            warning("Ardupilot Firmware: ${build} will be compiled and can take a while, please be patient")
            exec { "ardupilot_fw_build_${build}":
                user        => "mav",
                timeout     => 0,
                command     => "/srv/maverick/code/ardupilot/waf configure --board ${board} && /srv/maverick/code/ardupilot/waf ${build} >/srv/maverick/var/log/build/ardupilot-fw-${build}.build.log 2>&1",
                cwd         => "/srv/maverick/code/ardupilot",
                creates     => "/srv/maverick/code/ardupilot/${build}.elf",
            }
        }
    }
    
    define fwbuildmake ($build, $board) {
        $downvar = downcase($build)
        $buildvar = "ardupilotfw_${downvar}"
        $warningvar = getvar("${buildvar}")
        if $warningvar == "no" {
            warning("Arudpilot SITL Firmware: ${buildvar} will be compiled and can take a while, please be patient")
            exec { "ardupilot_fw_build_${build}":
                user        => "mav",
                timeout     => 0,
                command     => "/usr/bin/make -j${::processorcount} ${board} >/srv/maverick/var/log/build/ardupilot-fw-${build}.build.log 2>&1",
                cwd         => "/srv/maverick/code/dronekit-sitl/sitl-fw/${build}",
                creates     => "/srv/maverick/code/dronekit-sitl/sitl-fw/${build}/${build}.elf",
            }
        }
    }

    if $sitl and $ardupilot_buildsystem == "waf" {
        fwbuildwaf { "sitl_${ardupilot_type}":
            require     => Oncevcsrepo["git-ardupilot"],
            board       => "sitl",
            build       => $ardupilot_type,
        }
    } elsif $sitl and $ardupilot_buildsystem == "make" {
        fwbuildmake { "sitl_${ardupilot_type}": 
            require     => Oncevcsrepo["git-ardupilot"],
            board       => "sitl",
            build       => $ardupilot_type,
        }
    }

}