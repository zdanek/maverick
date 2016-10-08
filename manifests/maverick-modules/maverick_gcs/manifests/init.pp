class maverick_gcs (
    $skysense = false,
    $qgroundcontrol = false,
    $apmplanner2 = false,
) {
    
    if $skysense == true {
        class { "maverick_gcs::skysense": }
    }
    
}