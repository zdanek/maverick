class base::console {
    
    # Instead of using puppet template to fill in motd, instead use a systemd oneshot service so it refreshes on boot
    file { "/etc/systemd/system/maverick-motd.service":
        content     => template("base/maverick-motd.service.erb"),
        owner       => "root",
        group       => "root",
        mode        => "644",
        notify      => Exec["maverick-systemctl-daemon-reload"],
    }
    service { "maverick-motd":
        enable      => true
    }
    
    # Leave the ubuntu motd header in place as it usually contains the OS+kernel version, but remove the help stuff
    file { "/etc/update-motd.d/10-help-text":
        ensure      => absent
    }
    file { "/etc/update-motd.d/00-header":
        ensure      => absent
    }
    file { "/etc/legal":
        ensure      => absent
    }
    
    ### Colored Profile
    file { "/etc/profile.d/colorprompt.sh":
        content 	=> template("base/colorprompt.sh.erb"),
    }
    
    # Install screen that we use to access mavproxy and other consoles
    ensure_packages(["screen"])
    
}
