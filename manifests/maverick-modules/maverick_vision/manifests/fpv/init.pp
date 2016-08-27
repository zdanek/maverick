class maverick_vision::fpv::init (
    $type = "infrastructure", # 'infrastructure' for normal wifi, 'ap' to act as an Access Point, 'broadcast' for wifibroadcast, MUST match maverick_network::wireless::type
) {

    if $type == "infrastructure" {
        #class { "maverick-fpv::infrastructure": }
    } elsif $type == "ap" {
        class { "maverick-fpv::ap": }
    } elsif $type == "broadcast" {
        class { "maverick-fpv::broadcast": }
    }
    
}