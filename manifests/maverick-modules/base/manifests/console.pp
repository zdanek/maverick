class base::console {
    
    ### MOTD
    file { '/etc/motd':
        ensure      => present,
        owner       => 'root',
        group       => 'root',
        mode        => 644,
        content     => template('base/motd.erb'),
    }
    
    ### Colored Profile
    file { "/etc/profile.d/colorprompt.sh":
        content 	=> template("base/colorprompt.sh.erb"),
    }
    
}
