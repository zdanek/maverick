class maverick-dronekit::dev (
    $sitl = true,
    # $sitl_fw_branch = "Copter-3.3",
    $sitl_fw_branch = "master",
    #$sitl_fw_builds = ["ArduCopter", "ArduPlane", "APMrover2", "AntennaTracker"],
    $sitl_fw_builds = ["ArduCopter"], # only build copter by default
    $sitl_fw_run = "ArduCopter", # Which firmware will sitl run, must be part of $sitl_fw_builds
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
        venv_dir     => '/srv/maverick/.virtualenvs/dronekit-dev',
        owner        => 'mav',
        group        => 'mav',
        cwd          => '/srv/maverick/code/dronekit-dev',
        timeout      => 0,
    } ->
    vcsrepo { "/srv/maverick/code/dronekit-dev/dronekit-python":
        ensure		=> present,
        provider 	=> git,
        source		=> "http://github.com/dronekit/dronekit-python.git",
        revision	=> "master",
        owner		=> "mav",
        group		=> "mav",
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
            command     => "/usr/bin/make -j${::processorcount} sitl",
            cwd         => "/srv/maverick/code/dronekit-dev/ardupilot/${build}",
            creates     => "/srv/maverick/code/dronekit-dev/ardupilot/${build}/${build}.elf",
        }
    }
    # Install dronekit-sitl into dronekit-dev
    if $sitl == true {
        python::pip { 'pip-dronekit-sitl':
            pkgname     => 'dronekit-sitl',
            virtualenv  => '/srv/maverick/.virtualenvs/dronekit-dev',
            ensure      => present,
            owner       => 'mav',
            timeout     => 0,
        }
        python::pip { 'pip-mavproxy-sitl':
            pkgname     => 'MAVProxy',
            virtualenv  => '/srv/maverick/.virtualenvs/dronekit-dev',
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
        
        # This is needed for sitl run
        file { "/var/APM":
            ensure      => directory,
            owner       => "mav",
            group       => "mav",
            mode        => 755,
        }
        
        # Punch some holes in the firewall for sitl
        if defined(Class["::maverick-security"]) {
            maverick-security::firewall::firerule { "dev-sitl":
                ports       => [5770-5775],
                ips         => hiera("all_ips"),
                proto       => "tcp"
            }
        }
        
        file { "/etc/systemd/system/dev-sitl.service":
            content     => template("maverick-dronekit/dev-sitl.service.erb"),
            owner       => "root",
            group       => "root",
            mode        => 644
        } ->
        service { "dev-sitl":
            ensure      => running,
            enable      => true,
            require     => [ Exec["sitl_fw_build_${sitl_fw_run}"], Python::Pip['pip-mavproxy-sitl'] ],
        }
        
        file { "/etc/systemd/system/dev-mavproxy.service":
            content     => template("maverick-dronekit/dev-mavproxy.service.erb"),
            owner       => "root",
            group       => "root",
            mode        => 644
        } ->
        service { "dev-mavproxy":
            ensure      => running,
            enable      => true,
            require     => [ Service["dev-sitl"] ],
        }
        # Punch some holes in the firewall for sitl
        if defined(Class["::maverick-security"]) {
            maverick-security::firewall::firerule { "dev-mavproxy":
                ports       => [14560-14565],
                ips         => hiera("all_ips"),
                proto       => "udp"
            }
        }
        
    }

}