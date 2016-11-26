class maverick_gcs (
    $skysense = false,
    $qgroundcontrol = false,
    $apmplanner2 = false,
) {
    
    if $skysense == true {
        class { "maverick_gcs::skysense": }
    }
    
    if $qgroundcontrol == true {
        class { "maverick_gcs::qgroundcontrol": }
    }
    
    if $apmplanner2 == true {
        class { "maverick_gcs::apmplanner2": }
    }
    
}