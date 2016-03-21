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
    git::config { 'push.default':
        value => "simple",
        user => "mav"
    }

    # Pull maverick into it's final resting place
    file { "/srv/maverick/software/maverick":
        ensure 		=> directory,
        require		=> File["/srv/maverick/software"],
    mode		=> 755,
    owner		=> "mav",
    group		=> "mav",
    } ->
    vcsrepo { "/srv/maverick/software/maverick":
        ensure		=> present,
        provider 	=> git,
        source		=> "https://github.com/fnoop/maverick.git",
        revision	=> "master",
    owner		=> "mav",
    group		=> "mav",
    } ->
    exec { "gitfreeze-localconf":
        cwd         => "/srv/maverick/software/maverick",
        onlyif      => "/usr/bin/git ls-files -v conf/localconf.json |grep '^H'",
        command     => "/usr/bin/git update-index --assume-unchanged conf/localconf.json"
    }
    file { "/etc/profile.d/maverick-path.sh":
        ensure      => present,
        mode        => 644,
        owner       => "root",
        group       => "root",
        content     => "PATH=\$PATH:/srv/maverick/software/maverick",
    }
    exec { "sudoers-securepath":
        command     => '/bin/sed /etc/sudoers -i -r -e \'s#"$#:/srv/maverick/software/maverick"#\'',
        unless      => "/bin/grep 'secure_path' /etc/sudoers |/bin/grep 'maverick'"
    }
    
    # Add environment marker
    file { "/srv/maverick/.environment":
        ensure      => file,
        owner       => "mav",
        group       => "mav",
        mode        => 644,
        content     => $environment,
    }

}
