class base::console {
    
    ### MOTD
    #file { '/etc/motd':
    #    ensure      => present,
    #    owner       => 'root',
    #    group       => 'root',
    #    mode        => 644,
    #    content     => template('base/motd.erb'),
    #}
    
    # Instead of using puppet template to fill in motd, instead use a systemd oneshot service so it refreshes on boot
    file { "/srv/maverick/software/maverick/bin/maverick-info":
        ensure  => link,
        target  => "/srv/maverick/software/maverick/manifests/maverick-modules/base/files/maverick-info"
    }
    file { "/etc/systemd/system/maverick-motd.service":
        content     => template("base/maverick-motd.service.erb"),
        owner       => "root",
        group       => "root",
        mode        => 644,
        notify      => Exec["maverick-systemctl-daemon-reload"],
    }
    service { "maverick-motd":
        enable      => true
    }
    
    ### Colored Profile
    file { "/etc/profile.d/colorprompt.sh":
        content 	=> template("base/colorprompt.sh.erb"),
    }
    
    # Install screen that we use to access mavproxy and other consoles
    ensure_packages(["screen"])
    
}
