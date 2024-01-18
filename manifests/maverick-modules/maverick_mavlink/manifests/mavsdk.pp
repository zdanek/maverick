# @summary
#   Maverick_mavlink::Mavsdk class
#   This class installs MavSDK software.
#
# @example Declaring the class
#   This class is included from maverick_mavlink class and should not be included from elsewhere
#
class maverick_mavlink::mavsdk (
    $revision = "v2.0.1",
) {

    # Install px4 dev/build dependencies
    ensure_packages(["cmake", "build-essential", "colordiff", "astyle", "libcurl4-openssl-dev", "doxygen", "libgrpc-dev", "libgrpc++-dev"])

    # Install mavsdk
    if ! ("install_flag_mavsdk" in $installflags) {
        if $raspberry_present == "yes" {
            $atomic_environment = ["CXXFLAGS=-latomic"]
        } else {
	        $atomic_environment = undef
	    }
        oncevcsrepo { "git-mavsdk":
            gitsource   => "https://github.com/mavlink/MAVSDK.git",
            dest        => "/srv/maverick/var/build/mavsdk",
            revision    => $revision,
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
            environment => $atomic_environment,
            command     => "/usr/bin/cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DBUILD_MAVSDK_SERVER=ON -DSUPERBUILD=ON -Bbuild/default -H. -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/mavsdk -DDEPS_INSTALL_PATH=/srv/maverick/software/mavsdk -DCMAKE_INSTALL_RPATH=/srv/maverick/software/mavsdk/lib -j ${::processorcount} >/srv/maverick/var/log/build/mavsdk.cmake.out 2>&1",
            cwd         => "/srv/maverick/var/build/mavsdk",
            creates     => "/srv/maverick/var/build/mavsdk/build/default/Makefile",
            require     => [ Package["libgrpc-dev"], Package["libgrpc++-dev"] ],
        } ->
        exec { "mavsdk-build":
            user        => "mav",
            timeout     => 0,
            environment => ["LD_LIBRARY_PATH=/srv/maverick/var/build/mavsdk/build/default/third_party/install/lib"],
            command     => "/usr/bin/cmake --build build/default -j ${::processorcount} > /srv/maverick/var/log/build/mavsdk.cmake.out 2>&1",
            cwd         => "/srv/maverick/var/build/mavsdk",
            creates     => "/srv/maverick/var/build/mavsdk/build/default/src/core/libmavsdk.so",
            require     => [ Package["libgrpc-dev"], Package["libgrpc++-dev"] ],
        } ->
        exec { "mavsdk-install":
            user        => "mav",
            timeout     => 0,
            environment => ["LD_LIBRARY_PATH=/srv/maverick/var/build/mavsdk/build/default/third_party/install/lib"],
            command     => "/usr/bin/cmake --build build/default --target install >/srv/maverick/var/log/build/mavsdk.install.out 2>&1",
            cwd         => "/srv/maverick/var/build/mavsdk",
            creates     => "/srv/maverick/software/mavsdk/lib/libmavsdk.so",
        } ->
        file { "/srv/maverick/var/build/.install_flag_mavsdk":
            ensure      => present,
            owner       => "mav",
        }
    }

    install_python_module { "mavsdk-python":
        pkgname => "mavsdk",
        ensure  => present,
    }

}
