class maverick_gcs (
    $skysense = false,
    $qgroundcontrol = false,
    $apmplanner2 = false,
    $fcs = true,
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
    
    if $fcs == true {
        class { "maverick_gcs::fcs": }
    }
    
}