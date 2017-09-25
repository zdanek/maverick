class maverick_vision::vision_landing (
    $active = false,
    $vision_landing_source = "https://github.com/fnoop/vision_landing.git",
    $vision_landing_revision = "master",
) {

    # Ensure gstreamer resources are applied before this class
    require maverick_vision::gstreamer
    require maverick_vision::opencv
    require maverick_vision::aruco
    
    # Install vision_landing
    oncevcsrepo { "git-vision_landing":
        gitsource   => $vision_landing_source,
        dest        => "/srv/maverick/software/vision_landing",
        revision    => $vision_landing_revision,
        depth       => 0,
    } ->
    # Compile vision_landing
    exec { "vision_landing-compile":
        user        => mav,
        environment => ["CMAKE_PREFIX_PATH=/srv/maverick/software/opencv:/srv/maverick/software/aruco:/srv/maverick/software/librealsense"],
        cwd         => "/srv/maverick/software/vision_landing/src",
        command     => "/usr/bin/cmake -DCMAKE_MODULE_PATH=/srv/maverick/software/opencv . && make && make install",
        creates     => "/srv/maverick/software/vision_landing/track_targets",
        require     => [ Class["maverick_vision::opencv"], Class["maverick_vision::aruco"] ],
    } ->
    # Install systemd manifest
    file { "/etc/systemd/system/maverick-vision_landing.service":
        source      => "puppet:///modules/maverick_vision/vision_landing.service",
        owner       => root,
        group       => root,
    } ->
    # Place a default config file
    file { "/srv/maverick/data/config/vision/vision_landing.conf":
        owner       => "mav",
        group       => "mav",
        mode        => "644",
        content     => template("maverick_vision/vision_landing.conf.erb"),
        replace     => false,
    }
    
    # Create data and log directories
    file { ["/srv/maverick/data/vision/vision_landing", "/srv/maverick/var/log/vision/vision_landing"]:
        owner       => "mav",
        group       => "mav",
        mode        => "755",
        ensure      => directory,
    }

    # Activate or inactivate service
    if $active == true {
        service_wrapper { "maverick-vision_landing":
            ensure  => running,
            enable  => true,
            require => Exec["vision_landing-compile"]
        }
    } else {
        service_wrapper { "maverick-vision_landing":
            ensure  => stopped,
            enable  => false
        }
    }

}