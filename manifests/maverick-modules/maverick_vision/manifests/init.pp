class maverick_vision (
    $visiond = true,
    $gstreamer = true,
    $opencv = true,
    $mjpg_streamer = false,
    $aruco = true,
    $orb_slam2 = false,
    $vision_landing = true,
    $camera_streaming_daemon = true,
) {

    file { "/srv/maverick/data/config/vision":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    }
    
    if $visiond == true {
        class { "maverick_vision::visiond": 
            require => Class["maverick_vision::gstreamer"],
        }
    }

    if $gstreamer == true {
        class { "maverick_vision::gstreamer": }
    }

    if $mjpg_streamer == true  {
        class { "maverick_vision::mjpg-streamer": }
    }

    if $opencv == true {
        class { "maverick_vision::opencv": 
            require => Class["maverick_vision::gstreamer"],
        }
    }

    if $aruco == true {
        class { "maverick_vision::aruco":
            require => Class["maverick_vision::opencv"],
        }
    }

    if $orb_slam2 == true {
        class { "maverick_vision::orb_slam2": 
            require => Class["maverick_vision::opencv"],
        }
    }

    if $vision_landing == true {
        class { "maverick_vision::vision_landing": 
            require => Class["maverick_vision::aruco"],
        }
    }

    if $camera_streaming_daemon == true {
        class { "maverick_vision::camera_streaming_daemon": 
            require => Class["maverick_vision::gstreamer"],
        }
    }

}