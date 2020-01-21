class maverick_intelligence (
    $tensorflow = true,
    $pytorch = true,
    $openkai = false,
    $redtail = false,
) {
    
    if $tensorflow == true {
        class { "maverick_intelligence::tensorflow": }
    }
    
    if $pytorch == true {
        class { "maverick_intelligence::pytorch": }
    }

    if $openkai == true {
        class { "maverick_intelligence::openkai": }
    }
    
    if $redtail == true {
        class { "maverick_intelligence::redtail": }
    }

}
