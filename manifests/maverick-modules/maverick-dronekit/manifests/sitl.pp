class maverick-dronekit::sitl (
    # $sitl_fw_branch = "Copter-3.3",
    $sitl_fw_branch = "master",
    #$sitl_fw_builds = ["ArduCopter", "ArduPlane", "APMrover2", "AntennaTracker"],
    $sitl_fw_builds = ["ArduCopter"], # only build copter by default
    $sitl_fw_run = "ArduCopter", # Which firmware will sitl run, must be part of $sitl_fw_builds
    $sitl_dronekit_source = "http://github.com/dronekit/dronekit-python.git",
) {
    
    # Install a virtual environment for dronekit sitl
    file { "/srv/maverick/code/dronekit-sitl":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => 755,
    } ->
    python::virtualenv { '/srv/maverick/code/dronekit-sitl':
        ensure       => present,
        version      => 'system',
        systempkgs   => false,
        distribute   => true,
        venv_dir     => '/srv/maverick/.virtualenvs/dronekit-sitl',
        owner        => 'mav',
        group        => 'mav',
        cwd          => '/srv/maverick/code/dronekit-sitl',
        timeout      => 0,
    } ->
    file { "/srv/maverick/.virtualenvs/dronekit-sitl/lib/python2.7/no-global-site-packages.txt":
        ensure  => absent
    } ->
    oncevcsrepo { "git-sitl-dronekit-python":
        gitsource   => $sitl_dronekit_source,
        dest        => "/srv/maverick/code/dronekit-sitl/dronekit-python",
    }
    
    # Define function to build ardupilot firmwares, this is used for iteration if $sitl == true
    define sitl_fw_build ($build = $title) {
        $downvar = downcase($build)
        $buildvar = "ardupilotfw_${downvar}"
        $warningvar = getvar("${buildvar}")
        if $warningvar == "no" {
            warning("Arudpilot SITL Firmware: ${build} will be compiled and can take a while, please be patient")
        }
        exec { "sitl_fw_build_${build}":
            user        => "mav",
            timeout     => 0,
            command     => "/usr/bin/make -j${::processorcount} sitl >/srv/maverick/data/logs/build/sitl-fw-${build}.build.log 2>&1",
            cwd         => "/srv/maverick/code/dronekit-sitl/sitl-fw/${build}",
            creates     => "/srv/maverick/code/dronekit-sitl/sitl-fw/${build}/${build}.elf",
        }
    }
    
    # Install dronekit-sitl into dronekit-sitl
    python::pip { 'pip-dronekit-sitl':
        pkgname     => 'dronekit',
        virtualenv  => '/srv/maverick/.virtualenvs/dronekit-sitl',
        ensure      => present,
        owner       => 'mav',
        timeout     => 0,
    }
    python::pip { 'pip-dronekit-sitl-sitl':
        pkgname     => 'dronekit-sitl',
        virtualenv  => '/srv/maverick/.virtualenvs/dronekit-sitl',
        ensure      => present,
        owner       => 'mav',
        timeout     => 0,
    }
    python::pip { 'pip-mavproxy-sitl':
        pkgname     => 'MAVProxy',
        virtualenv  => '/srv/maverick/.virtualenvs/dronekit-sitl',
        ensure      => present,
        owner       => 'mav',
        timeout     => 0,
    }
        
    # Pull ardupilot to build firmwares for sitl, as pre-built firmwares aren't available for ARM.
    # For now just pull master, sort out releases/versions later
    oncevcsrepo { "git-sitl-fw":
        gitsource   => "https://github.com/ArduPilot/ardupilot.git",
        dest        => "/srv/maverick/code/dronekit-sitl/sitl-fw",
        revision	=> "${sitl_fw_branch}",
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
    
    # Punch some holes in the firewall for sitl, protect 5770 which mavproxy-sitl uses
    if defined(Class["::maverick-security"]) {
        maverick-security::firewall::firerule { "dev-sitl":
            ports       => [5771-5775],
            ips         => hiera("all_ips"),
            proto       => "tcp"
        }
    }
    
    file { "/srv/maverick/data/logs/mavproxy-sitl":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => 755,
    } ->

    file { "/srv/maverick/data/config/mavproxy-sitl.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        replace     => false, # initialize but don't overwrite in the future
        source      => "puppet:///modules/maverick-dronekit/mavproxy-sitl.conf",
    } ->
    file { "/srv/maverick/software/maverick/bin/mavproxy-sitl.sh":
        ensure      => link,
        target      => "/srv/maverick/software/maverick/manifests/maverick-modules/maverick-dronekit/files/mavproxy-sitl.sh",
    } ->
    file { "/etc/systemd/system/mavproxy-sitl.service":
        content     => template("maverick-dronekit/mavproxy-sitl.service.erb"),
        owner       => "root",
        group       => "root",
        mode        => 644,
        notify      => [ Exec["maverick-systemctl-daemon-reload"], Service["mavproxy-sitl"] ]
    } ->
    service { "dev-sitl":
        ensure      => running,
        enable      => true,
        require     => [ Exec["sitl_fw_build_${sitl_fw_run}"], Python::Pip['pip-mavproxy-sitl'], Exec["maverick-systemctl-daemon-reload"] ],
    }
    service { "mavproxy-sitl":
        ensure      => running,
        enable      => true,
        require       => [ Exec["maverick-systemctl-daemon-reload"], Service["dev-sitl"] ],
    }

    # Punch some holes in the firewall for sitl
    if defined(Class["::maverick-security"]) {
        maverick-security::firewall::firerule { "mavproxy-sitl":
            ports       => [14560-14565],
            ips         => hiera("all_ips"),
            proto       => "udp"
        }
    }

}