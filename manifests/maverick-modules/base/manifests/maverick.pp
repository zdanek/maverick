class base::maverick (
    $maverick_branch = "stable",
) {
   
   # Note: The mav user is setup in base::users
   
   file { ["/srv/", "/srv/maverick", "/srv/maverick/software", "/srv/maverick/code", "/srv/maverick/data", "/srv/maverick/data/logs", "/srv/maverick/config", "/srv/maverick/config/maverick", "/srv/maverick/config/maverick/local-nodes", "/srv/maverick/var", "/srv/maverick/var/build", "/srv/maverick/var/log", "/srv/maverick/var/log/build", "/srv/maverick/var/log/maverick", "/srv/maverick/var/run", "/srv/maverick/var/lib"]:
        ensure  => directory,
        owner   => "mav",
        group   => "mav",
        mode    => "755",
    }
    file { "/srv/maverick/.virtualenvs":
        ensure	=> directory,
        owner   => "mav",
        group   => "mav",
        mode    => "755",
    }
    
    # Setup git for the mav user
    include git
    # Lay down a default .gitconfig for mav user, don't overwrite modifications in the future
    file { "/srv/maverick/.gitconfig":
        source          => "puppet:///modules/base/mav_gitconfig",
        owner           => "mav",
        group           => "mav",
        mode            => "644",
        replace         => false,
    }

    # Pull maverick into it's final resting place
    oncevcsrepo { "git-maverick":
        gitsource   => "https://github.com/fnoop/maverick.git",
        dest        => "/srv/maverick/software/maverick",
	    revision    => $maverick_branch,
    }
    file { "/srv/maverick/config/maverick/localconf.json":
        source      => "puppet:///modules/base/maverick-localconf.json",
        mode        => "644",
        owner       => "mav",
        group       => "mav",
        replace     => false,
    } ->
    file { "/etc/profile.d/maverick-call.sh":
        ensure      => present,
        mode        => "644",
        owner       => "root",
        group       => "root",
        content     => "maverick() { /srv/maverick/software/maverick/bin/maverick \$1 \$2 \$3 \$4; . /etc/profile; }",
    }
    exec { "sudoers-securepath":
        command     => '/bin/sed /etc/sudoers -i -r -e \'s#"$#:/srv/maverick/software/maverick/bin"#\'',
        unless      => "/bin/grep 'secure_path' /etc/sudoers |/bin/grep 'maverick/bin'"
    }
    
    # Add environment marker
    file { "/srv/maverick/config/maverick/maverick-environment.conf":
        ensure      => file,
        owner       => "mav",
        group       => "mav",
        mode        => "644",
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
    file { "/srv/maverick/software/maverick/bin/wifi-setup":
        ensure  => link,
        target  => "/srv/maverick/software/maverick/manifests/maverick-modules/maverick_network/files/wifi-setup",
        require => Oncevcsrepo["git-maverick"],
    }
    
    # Remove old branch file
    exec { "move-oldbranchfile":
        command     => "/bin/mv /srv/maverick/config/maverick-branch.conf /srv/maverick/config/maverick",
        onlyif      => "/bin/ls /srv/maverick/config/maverick-branch.conf",
    } ->
    # Add maverick git branch config
    file { "/srv/maverick/config/maverick/maverick-branch.conf":
        owner   => "mav",
        group   => "mav",
        content => "MAVERICK_BRANCH=stable",
        replace => false,
    }
    
    # Ensure desktop config directory exists and prevent auto directory creation
    file { "/srv/maverick/.config":
        owner   => "mav",
        group   => "mav",
        mode    => "755",
        ensure  => directory,
    } ->
    file { "/srv/maverick/.config/user-dirs.conf":
        owner   => "mav",
        group   => "mav",
        mode    => "644",
        content => "enabled=False",
    }
    
    file { "/srv/maverick/.cache":
        ensure  => directory,
        owner   => "mav",
        group   => "mav",
    } ->
    file { "/srv/maverick/.cache/pip":
        ensure  => directory,
        owner   => "mav",
        group   => "mav",
        recurse => true,
    }
    
    # Create a 'firstboot' service that runs on boot and calls configure if required
    file { "/etc/systemd/system/maverick-firstboot.service":
        source      => "puppet:///modules/base/maverick-firstboot.service",
        owner       => "root",
        group       => "root",
        mode        => "644",
        notify      => Exec["maverick-systemctl-daemon-reload"],
    } ->
    service_wrapper { "maverick-firstboot": 
        enable      => true,
        ensure      => undef,
    }

}
