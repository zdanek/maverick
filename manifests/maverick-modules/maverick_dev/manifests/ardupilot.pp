class maverick_dev::ardupilot (
    $ardupilot_source = "https://github.com/ArduPilot/ardupilot.git",
    $ardupilot_setupstream = true,
    $ardupilot_upstream = "https://github.com/ArduPilot/ardupilot.git",
    $ardupilot_branch = "master", # eg. master, Copter-3.3, ArduPlane-release
    $ardupilot_board = "px4-v4",
    $ardupilot_buildsystem = "waf", # waf (Copter >=3.4) or make (Copter <3.3)
    $ardupilot_vehicle = "copter", # copter, plane or rover
    $sitl, # passed from init.pp
    $armeabi_packages = false, # needed to cross-compile firmware for actual FC boards
) {
    
    # Install ardupilot from git
    oncevcsrepo { "git-ardupilot":
        gitsource   => $ardupilot_source,
        dest        => "/srv/maverick/code/ardupilot",
        revision	=> $ardupilot_branch,
        submodules  => true,
    }
    ensure_packages(["make", "gawk", "g++", "zip", "genromfs", "python-empy"])
    if $armeabi_packages == true {
        ensure_packages(["gcc-arm-none-eabi", "binutils-arm-none-eabi", "gdb-arm-none-eabi", "libnewlib-arm-none-eabi", "libstdc++-arm-none-eabi-newlib"], {'ensure'=>'present'})
    } else {
        ensure_packages(["gcc-arm-none-eabi", "binutils-arm-none-eabi", "gdb-arm-none-eabi", "libnewlib-arm-none-eabi", "libstdc++-arm-none-eabi-newlib"], {'ensure'=>'purged'})
    }
    
    # If a custom ardupilot repo is specified, configure the upstream automagically
    exec { "ardupilot_setupstream":
        command     => "/usr/bin/git remote add upstream ${ardupilot_upstream}",
        unless      => "/usr/bin/git remote -v | /bin/grep ${ardupilot_upstream}",
        cwd         => "/srv/maverick/code/ardupilot",
        require     => Oncevcsrepo["git-ardupilot"],
    }
    
    # Compile SITL
    if $sitl and $ardupilot_buildsystem == "waf" {
        maverick_dev::fwbuildwaf { "sitl_${ardupilot_vehicle}":
            require     => [ Oncevcsrepo["git-ardupilot"], Exec["ardupilot_setupstream"] ],
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
        } elsif $ardupilot_vehicle == "sub" {
            $ardupilot_type = "ArduSub"
        } elsif $ardupilot_vehicle == "antennatracker" {
            $ardupilot_type = "AntennaTracker"
        }
        fwbuildmake { "sitl_${ardupilot_vehicle}": 
            require     => [ Oncevcsrepo["git-ardupilot"], Exec["ardupilot_setupstream"] ],
            board       => "sitl",
            build       => $ardupilot_type,
        }
    }

}
