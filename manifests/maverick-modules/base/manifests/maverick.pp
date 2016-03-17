class base::maverick {
   
    file { "/srv":
        ensure  => directory,
        owner   => "mav",
        group   => "mav",
        mode    => 755
    } ->
    file { "/srv/software":
        ensure	=> directory,
        owner   => "mav",
        group   => "mav",
        mode    => 755
    } ->
    file { "/srv/code":
        ensure	=> directory,
        owner   => "mav",
        group   => "mav",
        mode    => 755
    } ->
    file { "/srv/data":
        ensure	=> directory,
        owner   => "mav",
        group   => "mav",
        mode    => 755
    }
    
}
