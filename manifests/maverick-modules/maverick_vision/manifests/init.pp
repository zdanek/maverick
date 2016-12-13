class maverick_vision (
    $visiond = true,
    $gstreamer = true,
    $opencv = true,
    $mjpg_streamer = false,
    $visiond_state = undef,
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
    
}