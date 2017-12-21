class maverick_intelligence (
    $tensorflow = true,
    $openkai = false,
) {
    
    if $tensorflow == true {
        class { "maverick_intelligence::tensorflow": }
    }
    
    if $openkai == true {
        class { "maverick_intelligence::openkai": }
    }
    
}