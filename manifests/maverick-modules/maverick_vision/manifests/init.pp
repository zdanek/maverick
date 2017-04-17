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
        class { "maverick_vision::visiond": }
    }

    if $gstreamer == true {
        class { "maverick_vision::gstreamer": }
    }

    if $mjpg_streamer == true  {
        class { "maverick_vision::mjpg-streamer": }
    }

    if $opencv == true {
        class { "maverick_vision::opencv": }
    }

    if $aruco == true {
        class { "maverick_vision::aruco": }
    }

    if $orb_slam2 == true {
        class { "maverick_vision::orb_slam2": }
    }

    if $vision_landing == true {
        class { "maverick_vision::vision_landing": }
    }

    if $camera_streaming_daemon == true {
        class { "maverick_vision::camera_streaming_daemon": }
    }
    
}