class base::maverick {
   
   # Note: The mav user is setup in base::users
   
    file { "/srv":
        ensure  => directory,
        owner   => "mav",
        group   => "mav",
        mode    => 755
    }
    file { "/srv/maverick":
        ensure  => directory,
        owner   => "mav",
        group   => "mav",
        mode    => 755
    }
    file { "/srv/maverick/software":
        ensure	=> directory,
        owner   => "mav",
        group   => "mav",
        mode    => 755
    }
    file { "/srv/maverick/code":
        ensure	=> directory,
        owner   => "mav",
        group   => "mav",
        mode    => 755
    }
    file { "/srv/maverick/data":
        ensure	=> directory,
        owner   => "mav",
        group   => "mav",
        mode    => 755
    }
    file { "/srv/maverick/data/logs":
        ensure	=> directory,
        owner   => "mav",
        group   => "mav",
        mode    => 755
    }
    file { "/srv/maverick/var":
        ensure	=> directory,
        owner   => "mav",
        group   => "mav",
        mode    => 755
    }
    file { "/srv/maverick/var/build":
        ensure	=> directory,
        owner   => "mav",
        group   => "mav",
        mode    => 755
    }
    file { "/srv/maverick/var/log":
        ensure	=> directory,
        owner   => "mav",
        group   => "mav",
        mode    => 755
    }
    file { "/srv/maverick/var/log/build":
        ensure	=> directory,
        owner   => "mav",
        group   => "mav",
        mode    => 755
    }
    file { "/srv/maverick/var/run":
        ensure	=> directory,
        owner   => "mav",
        group   => "mav",
        mode    => 755
    }
    file { "/srv/maverick/data/config":
        ensure	=> directory,
        owner   => "mav",
        group   => "mav",
        mode    => 755
    }
    file { "/srv/maverick/.virtualenvs":
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
            value       => $git_username,
            user        => "mav",
            require     => File["/srv/maverick"]
        }
    } 
    $git_email = hiera('git_email')
    if $git_email {
        git::config { 'user.email':
            value       => $git_email,
            user        => "mav",
            require     => File["/srv/maverick"]
        }
    }
    git::config { 'credential.helper':
        value       => 'cache --timeout=86400',
        user        => "mav",
        require     => File["/srv/maverick"]
    }
    git::config { 'push.default':
        value       => "simple",
        user        => "mav",
        require     => File["/srv/maverick"]
    }

    # Pull maverick into it's final resting place
    oncevcsrepo { "git-maverick":
        gitsource   => "https://github.com/fnoop/maverick.git",
        dest        => "/srv/maverick/software/maverick",
    } ->
    exec { "gitfreeze-localconf":
        cwd         => "/srv/maverick/software/maverick",
        onlyif      => "/usr/bin/git ls-files -v conf/localconf.json |grep '^H'",
        command     => "/usr/bin/git update-index --assume-unchanged conf/localconf.json"
    } ->
    file { "/srv/maverick/data/config/maverick":
        ensure      => link,
        target      => "/srv/maverick/software/maverick/conf",
    }

    file { "/etc/profile.d/maverick-call.sh":
        ensure      => present,
        mode        => 644,
        owner       => "root",
        group       => "root",
        content     => "maverick() { /srv/maverick/software/maverick/bin/maverick \$1 \$2 \$3 \$4; . /etc/profile; }",
    }
    exec { "sudoers-securepath":
        command     => '/bin/sed /etc/sudoers -i -r -e \'s#"$#:/srv/maverick/software/maverick/bin"#\'',
        unless      => "/bin/grep 'secure_path' /etc/sudoers |/bin/grep 'maverick/bin'"
    }
    
    # Add environment marker
    file { "/srv/maverick/.environment":
        ensure      => file,
        owner       => "mav",
        group       => "mav",
        mode        => 644,
        content     => $environment,
    }

    # Start a concat file for maverick paths
    concat { "/etc/profile.d/maverick-path.sh":
        ensure      => present,
    }
    concat::fragment { "maverickpath-base":
        target      => "/etc/profile.d/maverick-path.sh",
        order       => 1,
        content     => "export PATH=\$PATH:/srv/maverick/software/maverick/bin",
    }
    
    # Create symlinks for maverick subcommands
    file { "/srv/maverick/software/maverick/bin/maverick-info":
        ensure  => link,
        target  => "/srv/maverick/software/maverick/manifests/maverick-modules/base/files/maverick-info",
        require => Oncevcsrepo["git-maverick"],
    }
    file { "/srv/maverick/software/maverick/bin/maverick-netinfo":
        ensure  => link,
        target  => "/srv/maverick/software/maverick/manifests/maverick-modules/maverick_network/files/maverick-netinfo",
        require => Oncevcsrepo["git-maverick"],
    }
    
}
