class maverick-dronekit (
    $dronekit_la_revision = "master",
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
    }

}