class maverick_web (
    $cloud9 = true,
    $nodejs = true,
) {
    
    if $nodejs == true {
        class { "maverick_web::nodejs": }
    }
    
    if $cloud9 == true {
        class { "maverick_web::cloud9": }
    }
    
}