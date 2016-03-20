class base::maverick {
   
    file { "/srv":
        ensure  => directory,
        owner   => "mav",
        group   => "mav",
        mode    => 755
    } ->
    file { "/srv/maverick":
        ensure  => directory,
        owner   => "mav",
        group   => "mav",
        mode    => 755
    } ->
    file { "/srv/maverick/software":
        ensure	=> directory,
        owner   => "mav",
        group   => "mav",
        mode    => 755
    } ->
    file { "/srv/maverick/code":
        ensure	=> directory,
        owner   => "mav",
        group   => "mav",
        mode    => 755
    } ->
    file { "/srv/maverick/data":
        ensure	=> directory,
        owner   => "mav",
        group   => "mav",
        mode    => 755
    }

    # Setup git for the mav user
    include git
    $git_username = hiera('git_username')
    if $git_username {
        git::config { 'user.name':
            value => $git_username,
            user => "mav",
        }
    } 
    $git_email = hiera('git_email')
    if $git_email {
        git::config { 'user.email':
            value => $git_email,
            user => "mav",
        }
    }
    git::config { 'credential.helper':
        value => 'cache --timeout=3600',
        user => "mav",
    }

}
