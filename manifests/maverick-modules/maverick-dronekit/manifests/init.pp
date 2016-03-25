class maverick-dronekit (
) {
    
    # Install dronekit through pip globally
    python::pip { 'pip-dronekit' :
        pkgname       => 'dronekit',
        ensure        => present,
        owner         => 'root',
        timeout       => 1800,
    }

}