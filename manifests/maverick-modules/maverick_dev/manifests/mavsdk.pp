class maverick_dev::mavsdk (
) {

    # Install px4 dev/build dependencies
    ensure_packages(["cmake", "build-essential", "colordiff", "astyle", "libcurl4-openssl-dev", "doxygen"])

    # Install mavsdk
    if ! ("install_flag_mavsdk" in $installflags) {
        oncevcsrepo { "git-mavsdk":
            gitsource   => "https://github.com/mavlink/MAVSDK.git",
            dest        => "/srv/maverick/var/build/mavsdk",
            submodules  => true,
        } ->
        file { ["/srv/maverick/var/build/mavsdk/build", "/srv/maverick/var/build/mavsdk/build/default"]:
            ensure      => directory,
            owner       => "mav",
            group       => "mav",
            mode        => "755",
        } ->
        exec { "mavsdk-prepbuild":
            user        => "mav",
            timeout     => 0,
            command     => "/usr/bin/cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DBUILD_BACKEND=ON -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/mavsdk ../.. >/srv/maverick/var/log/build/mavsdk.cmake.out 2>&1",
            cwd         => "/srv/maverick/var/build/mavsdk/build/default",
            creates     => "/srv/maverick/var/build/mavsdk/build/default/Makefile",
        } ->
        exec { "mavsdk-build":
            user        => "mav",
            timeout     => 0,
            command     => "/usr/bin/make -j${::processorcount} >/srv/maverick/var/log/build/mavsdk.build.out 2>&1",
            cwd         => "/srv/maverick/var/build/mavsdk/build/default",
            creates     => "/srv/maverick/var/build/mavsdk/build/default/src/core/libmavsdk.so",
        } ->
        exec { "mavsdk-install":
            user        => "mav",
            timeout     => 0,
            command     => "/usr/bin/make install >/srv/maverick/var/log/build/mavsdk.install.out 2>&1",
            cwd         => "/srv/maverick/var/build/mavsdk/build/default",
            creates     => "/srv/maverick/software/mavsdk/lib/libmavsdk.so",
        } ->
        file { "/srv/maverick/var/build/.install_flag_mavsdk":
            ensure      => present,
            owner       => "mav",
        }
    }

}
