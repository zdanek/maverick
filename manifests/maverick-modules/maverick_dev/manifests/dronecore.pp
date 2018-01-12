class maverick_dev::dronecore (
) {

    # Install px4 dev/build dependencies
    ensure_packages(["cmake", "build-essential", "colordiff", "astyle", "git", "libcurl4-openssl-dev", "doxygen"])

    # Install dronecore
    if ! ("install_flag_dronecore" in $installflags) {
        oncevcsrepo { "git-dronecore":
            gitsource   => "https://github.com/dronecore/DroneCore.git",
            dest        => "/srv/maverick/var/build/dronecore",
            submodules  => true,
        } -> exec { "dronecore-make":
            environment => ["BUILD_TYPE=Release", "INSTALL_PREFIX=/srv/maverick/software/dronecore"],
            command     => "/usr/bin/make default install >/srv/maverick/var/log/build/dronecore.make.log 2>&1",
            user        => "mav",
            timeout     => 0,
            cwd         => "/srv/maverick/var/build/dronecore",
            creates     => "/srv/maverick/software/dronecore/lib/libdronecore.so",
        } ->
        file { "/srv/maverick/var/build/.install_flag_dronecore":
            ensure      => present,
            owner       => "mav",
            group       => "mav",
            mode        => "0644",
        }
    }

}