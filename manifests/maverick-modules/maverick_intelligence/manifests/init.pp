class maverick_intelligence (
    $tensorflow = true,
) {
    
    if $tensorflow == true {
        class { "maverick_intelligence::tensorflow": }
    }
    
}