class maverick-dronekit (
    $dronekit_la_revision = "master",
    $sitl = false, # Don't install sitl setup by default, dev env will specify it
    $fc = true, # Do install flight controller setup by default 
) {
    
    # Install dronekit through pip globally
    python::pip { 'pip-dronekit' :
        pkgname       => 'dronekit',
        ensure        => present,
        owner         => 'root',
        timeout       => 1800,
    }
    
    # Install dronekit-la (log analyzer)
    vcsrepo { "/srv/maverick/software/dronekit-la":
        ensure		=> present,
        provider 	=> git,
        source		=> "https://github.com/dronekit/dronekit-la.git",
        revision	=> "${dronekit_la_revision}",
        owner		=> "mav",
        group		=> "mav",
        submodules  => false
    } ->
    # Compile dronekit-la
    exec { "compile-dronekit-la":
        command     => "/usr/bin/make -j${::processorcount} >/srv/maverick/data/logs/build/dronekit-la.build.log 2>&1",
        creates     => "/srv/maverick/software/dronekit-la/dronekit-la",
        user        => "mav",
        cwd         => "/srv/maverick/software/dronekit-la",
        timeout     => 0,
    }
    
    if $sitl {
        class { "maverick-dronekit::sitl": }
    }
    
    if $fc {
        class { "maverick-dronekit::fc": }
    }
    
    file { "/srv/maverick/code/dronekit-apps":
        ensure      => "directory",
        owner       => "mav",
        group       => "mav",
        mode        => 755,
    }
    
    # Install rmackay9 red balloon finder
    vcsrepo { "/srv/maverick/code/dronekit-apps/red-balloon-finder":
        ensure		=> present,
        provider 	=> git,
        source		=> "https://github.com/rmackay9/ardupilot-balloon-finder.git",
        revision	=> "master",
        owner		=> "mav",
        group		=> "mav",
    }
    # Install djnugent precision landing
    vcsrepo { "/srv/maverick/code/dronekit-apps/precision-landing":
        ensure		=> present,
        provider 	=> git,
        source		=> "https://github.com/djnugent/Precland.git",
        revision	=> "master",
        owner		=> "mav",
        group		=> "mav",
    }
    
    
}