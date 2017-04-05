class maverick_vision::vision_landing (
    $active = false,
) {

    # Install vision_landing
    oncevcsrepo { "git-vision_landing":
        gitsource   => "https://github.com/fnoop/vision_landing.git",
        dest        => "/srv/maverick/software/vision_landing",
    } ->
    # Compile vision_landing
    exec { "vision_landing-compile":
        user        => mav,
        environment => ["CMAKE_PREFIX_PATH=/srv/maverick/software/opencv:/srv/maverick/software/aruco"],
        cwd         => "/srv/maverick/software/vision_landing/src",
        command     => "/usr/bin/cmake -DCMAKE_MODULE_PATH=/srv/maverick/software/opencv . && make && make install",
        creates     => "/srv/maverick/software/vision_landing/track_targets",
        require     => [ File["/srv/maverick/var/build/.install_flag_opencv"], File["/srv/maverick/var/build/.install_flag_aruco"] ],
    } ->
    # Install systemd manifest
    file { "/etc/systemd/system/maverick-vision_landing.service":
        source      => "puppet:///modules/maverick_vision/vision_landing.service",
        owner       => root,
        group       => root,
    }
    
    # Activate or inactivate service
    if $active == true {
        service { "maverick-vision_landing":
            ensure  => running,
            enable  => true,
            require => Exec["vision_landing-compile"]
        }
    } else {
        service { "maverick-vision_landing":
            ensure  => stopped,
            enable  => false
        }
    }

}