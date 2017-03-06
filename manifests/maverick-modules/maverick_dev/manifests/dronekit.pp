class maverick_dev::dronekit (
    $vision_landing_active = false,
) {
        
    file { "/srv/maverick/code/dronekit-apps":
        ensure      => "directory",
        owner       => "mav",
        group       => "mav",
        mode        => 755,
    }
    
    # Install rmackay9 red balloon finder
    oncevcsrepo { "git-red-balloon-finder":
        gitsource   => "https://github.com/rmackay9/ardupilot-balloon-finder.git",
        dest        => "/srv/maverick/code/dronekit-apps/red-balloon-finder",
    }
    # Install vision_landing
    oncevcsrepo { "git-vision_landing":
        gitsource   => "https://github.com/fnoop/vision_landing.git",
        dest        => "/srv/maverick/code/dronekit-apps/vision_landing",
    } ->
    # Compile vision_landing
    exec { "vision_landing-compile":
        user        => mav,
        environment => ["CMAKE_PREFIX_PATH=/srv/maverick/software/opencv:/srv/maverick/software/aruco"],
        cwd         => "/srv/maverick/code/dronekit-apps/vision_landing/src",
        command     => "/usr/bin/cmake . && make && make install",
        creates     => "/srv/maverick/code/dronekit-apps/vision_landing/track_targets",
        require     => File["/srv/maverick/var/build/.install_flag_opencv"],
    } ->
    # Install systemd manifest
    file { "/etc/systemd/system/vision_landing.service":
        source      => "puppet:///modules/maverick_vision/vision_landing.service",
        owner       => root,
        group       => root,
    }
    
    # Activate or inactivate service
    if $vision_landing_active {
        service { "vision_landing":
            ensure  => running,
            enable  => true,
            require => Exec["vision_landing-compile"]
        }
    } else {
        service { "vision_landing":
            ensure  => stopped,
            enable  => false
        }
    }

}