class maverick-dronekit::dev (
    $sitl = true,
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

    # Install dronekit-sitl into dronekit-dev
    if $sitl == true {
        python::pip { 'pip-dronekit-sitl':
            pkgname     => 'dronekit-sitl',
            virtualenv  => '/srv/maverick/virtualenvs/dronekit-dev',
            ensure      => present,
            owner       => 'mav',
            timeout     => 0,
        }
    }
    
    # Pull ardupilot into software to build firmwares for sitl
    # For now just pull master, sort out releases/versions later
    vcsrepo { "/srv/maverick/code/dronekit-dev/ardupilot":
        ensure		=> present,
        provider 	=> git,
        source		=> "https://github.com/ArduPilot/ardupilot.git",
        revision	=> "master",
        owner		=> "mav",
        group		=> "mav",
    }
    ensure_packages(["make", "gawk", "g++", "arduino-core"])
    
    
}