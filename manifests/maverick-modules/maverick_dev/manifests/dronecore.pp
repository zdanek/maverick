class maverick_dev::dronecore (
) {

    # Install px4 dev/build dependencies
    ensure_packages(["cmake", "build-essential", "colordiff", "astyle", "libcurl4-openssl-dev", "doxygen"])

    # Install dronecore
    if ! ("install_flag_dronecore" in $installflags) {
        oncevcsrepo { "git-dronecore":
            gitsource   => "https://github.com/dronecore/DroneCore.git",
            dest        => "/srv/maverick/var/build/dronecore",
            submodules  => true,
        } ->
        file { ["/srv/maverick/var/build/dronecore/build", "/srv/maverick/var/build/dronecore/build/default"]:
            ensure      => directory,
            owner       => "mav",
            group       => "mav",
            mode        => "755",
        } ->
        exec { "dronecore-prepbuild":
            user        => "mav",
            timeout     => 0,
            command     => "/usr/bin/cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/dronecore -DBUILD_BACKEND=ON ../.. >/srv/maverick/var/log/build/dronecore.cmake.out 2>&1",
            cwd         => "/srv/maverick/var/build/dronecore/build/default",
            creates     => "/srv/maverick/var/build/dronecore/build/default/Makefile",
        } ->
        exec { "dronecore-build":
            user        => "mav",
            timeout     => 0,
            command     => "/usr/bin/make -j${::processorcount} >/srv/maverick/var/log/build/dronecore.build.out 2>&1",
            cwd         => "/srv/maverick/var/build/dronecore/build/default",
            creates     => "/srv/maverick/var/build/dronecore/build/default/aruco_tracker",
        } ->
        exec { "dronecore-install":
            user        => "mav",
            timeout     => 0,
            command     => "/usr/bin/make install >/srv/maverick/var/log/build/dronecore.install.out 2>&1",
            cwd         => "/srv/maverick/var/build/dronecore/build/default",
            creates     => "/srv/maverick/software/dronecore/lib/libdronecore.so",
        } ->
        file { "/srv/maverick/var/build/.install_flag_dronecore":
            ensure      => present,
            owner       => "mav",
        }
    }

}
