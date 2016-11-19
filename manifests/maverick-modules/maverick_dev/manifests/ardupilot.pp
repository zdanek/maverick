class maverick_dev::ardupilot (
    $ardupilot_source = "https://github.com/ArduPilot/ardupilot.git",
    $ardupilot_setupstream = true,
    $ardupilot_upstream = "https://github.com/ArduPilot/ardupilot.git",
    $ardupilot_branch = "master", # eg. master, Copter-3.3, ArduPlane-release
    $ardupilot_board = "px4-v4",
    $ardupilot_buildsystem = "waf", # waf (Copter >=3.4) or make (Copter <3.3)
    $ardupilot_vehicle = "copter", # copter, plane or rover
    $sitl, # passed from init.pp
) {
    
    # Install ardupilot from git
    oncevcsrepo { "git-ardupilot":
        gitsource   => $ardupilot_source,
        dest        => "/srv/maverick/code/ardupilot",
        revision	=> $ardupilot_branch,
        submodules  => true,
    }
    ensure_packages(["make", "gawk", "g++", "arduino-core", "gcc-arm-none-eabi", "binutils-arm-none-eabi", "gdb-arm-none-eabi", "genromfs", "python-empy"])
    # Waf build requires python future
    python::pip { 'pip-future':
        pkgname     => 'future',
        ensure      => present,
    }
    
    # If a custom ardupilot repo is specified, configure the upstream automagically
    if $ardupilot_source != $ardupilot_upstream and $ardupilot_setupstream {
        exec { "ardupilot_setupstream":
            command     => "/usr/bin/git remote add upstream ${ardupilot_upstream}",
            unless      => "/usr/bin/git remote -v | /bin/grep ${ardupilot_upstream}",
            cwd         => "/srv/maverick/code/ardupilot",
        }
    }
    
    # Define function to build ardupilot firmwares using new waf system
    define fwbuildwaf ($build, $board) {
        if ! ("${board}/bin/ardu${build}" in $waffiles) {
            warning("Ardupilot Firmware: ${build} will be compiled and can take a while, please be patient")
            exec { "ardupilotfw_${board}_${build}":
                user        => "mav",
                timeout     => 0,
                command     => "/srv/maverick/code/ardupilot/waf configure --board ${board} && /srv/maverick/code/ardupilot/waf ${build} >/srv/maverick/var/log/build/ardupilot-fw-${build}.build.log 2>&1",
                cwd         => "/srv/maverick/code/ardupilot",
                creates     => "/srv/maverick/code/ardupilot/${build}.elf",
                require     => Python::Pip['pip-future']
            }
        }
    }
    
    # Define function to build ardupilot firmwares using old make system
    define fwbuildmake ($build, $board) {
        $downvar = downcase($build)
        $buildvar = "ardupilotfw_${downvar}"
        $warningvar = getvar("${buildvar}")
        if $warningvar == "no" {
            warning("Arudpilot Firmware: ${buildvar} will be compiled and can take a while, please be patient")
            exec { "ardupilotfw_${board}_${build}":
                user        => "mav",
                timeout     => 0,
                command     => "/usr/bin/make -j${::processorcount} ${board} >/srv/maverick/var/log/build/ardupilot-fw-${build}.build.log 2>&1",
                cwd         => "/srv/maverick/code/ardupilot/${build}",
                creates     => "/srv/maverick/code/ardupilot/${build}/${build}.elf",
            }
        }
    }

    # Compile SITL
    if $sitl and $ardupilot_buildsystem == "waf" {
        fwbuildwaf { "sitl_${ardupilot_vehicle}":
            require     => Oncevcsrepo["git-ardupilot"],
            board       => "sitl",
            build       => $ardupilot_vehicle,
        }
    } elsif $sitl and $ardupilot_buildsystem == "make" {
        if $ardupilot_vehicle == "copter" {
            $ardupilot_type = "ArduCopter"
        } elsif $ardupilot_vehicle == "plane" {
            $ardupilot_type = "ArduPlane"
        } elsif $ardupilot_vehicle == "rover" {
            $ardupilot_type = "ArduRover"
        }
        fwbuildmake { "sitl_${ardupilot_vehicle}": 
            require     => Oncevcsrepo["git-ardupilot"],
            board       => "sitl",
            build       => $ardupilot_type,
        }
    }

}
