class maverick_intelligence (
    $tensorflow = true,
    $openkai = false,
    $redtail = false,
) {
    
    if $tensorflow == true {
        class { "maverick_intelligence::tensorflow": }
    }
    
    if $openkai == true {
        class { "maverick_intelligence::openkai": }
    }
    
    if $redtail == true {
        class { "maverick_intelligence::redtail": }
    }

}