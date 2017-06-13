class maverick_vision::collision_avoidance (
    $source = "https://github.com/01org/collision-avoidance-library.git",
    $revision = "master",
    $active = false,
) {
    
    # Ensure package dependencies are installed
    ensure_packages(["cmake", "libglm-dev", "doxygen", "libusb-1.0-0-dev", "libglfw3-dev"])
    
    if ! ("install_flag_collision_avoidance" in $installflags) {
        # Install collision_avoidance
        oncevcsrepo { "git-collision_avoidance":
            gitsource   => $source,
            dest        => "/srv/maverick/var/build/collision_avoidance",
            revision    => $revision,
            submodules  => true,
        } ->
        file { "/srv/maverick/var/build/collision_avoidance/build":
            ensure      => directory,
            owner       => "mav",
            group       => "mav",
            mode        => "755",
        } ->
        # Compile collision_avoidance
        exec { "collision_avoidance-compile":
            user        => mav,
            environment => ["CMAKE_PREFIX_PATH=/srv/maverick/software/librealsense"],
            cwd         => "/srv/maverick/var/build/collision_avoidance/build",
            command     => "/usr/bin/cmake .. -DCMAKE_PREFIX_PATH=/srv/maverick/software/librealsense -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/collision_avoidance -DCMAKE_INSTALL_RPATH=/srv/maverick/software/librealsense/lib:/srv/maverick/software/collision_avoidance/lib -DWITH_REALSENSE=ON -DWITH_TOOLS=ON -DWITH_SAMPLES=ON && make && make install",
            creates     => "/srv/maverick/software/collision_avoidance/bin/coav-control",
            require     => Class["maverick_hardware::peripheral::realsense"],
        } ->
        file { "/srv/maverick/var/build/.install_flag_collision_avoidance":
            owner       => "mav",
            group       => "mav",
            ensure      => present,
            mode        => "755",
        }
    }
 
    file { "/srv/maverick/data/config/vision/coav.conf":
        source          => "puppet:///modules/maverick_vision/coav.conf",
        replace         => false,
        owner           => "mav",
        group           => "mav",
    }
    file { "/etc/systemd/system/maverick-coav.service":
        owner           => "root",
        group           => "root",
        mode            => "644",
        source          => "puppet:///modules/maverick_vision/coav.service",
        notify          => Exec["maverick-systemctl-daemon-reload"],
        before          => Service["maverick-coav"],
    }
    
    if $active == true {
        service_wrapper { "maverick-coav":
            ensure      => running,
            enable      => true,
        }
    } else {
        service_wrapper { "maverick-coav":
            ensure      => stopped,
            enable      => false,
        }
    }
    
}