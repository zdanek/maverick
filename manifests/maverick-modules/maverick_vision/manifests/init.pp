class maverick_vision (
    $visiond = true,
    $gstreamer = true,
    $opencv = true,
    $mjpg_streamer = false,
    $aruco = true,
    $orbslam2 = false,
    $vision_landing = true,
) {

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

    if $orbslam2 == true {
        class { "maverick_vision::orbslam2": }
    }

    if $vision_landing == true {
        class { "maverick_vision::vision_landing": }
    }

}