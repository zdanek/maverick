class maverick-dronekit::dev (
    $sitl = true,
    $sitl_fw_branch = "master",
    $sitl_fw_builds = ["ArduCopter", "ArduPlane", "APMrover2", "AntennaTracker"],
    #$sitl_fw_builds = ["ArduCopter"] # only build copter by default
) {
    
    # Install a virtual environment for dronekit dev
    file { "/srv/maverick/code/dronekit-dev":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => 755,
    } ->
    python::virtualenv { '/srv/maverick/code/dronekit-dev':
        ensure       => present,
        version      => 'system',
        systempkgs   => false,
        distribute   => true,
        venv_dir     => '/srv/maverick/virtualenvs/dronekit-dev',
        owner        => 'mav',
        group        => 'mav',
        cwd          => '/srv/maverick/code/dronekit-dev',
        timeout      => 0,
    }

    # Define function to build ardupilot firmwares, this is used for iteration if $sitl == true
    define sitl_fw_build ($build = $title) {
        $downvar = downcase($build)
        $buildvar = "ardupilotfw_${downvar}"
        $warningvar = getvar("${buildvar}")
        if $warningvar == "no" {
            warning("Arudpilot Firmware: ${build} will be compiled and can take a while, please be patient")
        }
        exec { "sitl_fw_build_${build}":
            user        => "mav",
            timeout     => 0,
            command     => "/usr/bin/make -j${::processorcount} linux",
            cwd         => "/srv/maverick/code/dronekit-dev/ardupilot/${build}",
            creates     => "/srv/maverick/code/dronekit-dev/ardupilot/${build}/${build}.elf",
        }
    }
    # Install dronekit-sitl into dronekit-dev
    if $sitl == true {
        python::pip { 'pip-dronekit-sitl':
            pkgname     => 'dronekit-sitl',
            virtualenv  => '/srv/maverick/virtualenvs/dronekit-dev',
            ensure      => present,
            owner       => 'mav',
            timeout     => 0,
        }

        # Pull ardupilot to build firmwares for sitl, as pre-built firmwares aren't available for ARM.
        # For now just pull master, sort out releases/versions later
        vcsrepo { "/srv/maverick/code/dronekit-dev/ardupilot":
            ensure		=> present,
            provider 	=> git,
            source		=> "https://github.com/ArduPilot/ardupilot.git",
            revision	=> "${sitl_fw_branch}",
            owner		=> "mav",
            group		=> "mav",
        }
        ensure_packages(["make", "gawk", "g++", "arduino-core"])
        
        # Build specified firmwares iteratively
        sitl_fw_build { $sitl_fw_builds: }
        
    }

}