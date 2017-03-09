class maverick_desktop (
    $enable = false,
) {
    
    ### Desktop is disabled by default and must be specifically enabled
    ### This assumes that the desktop is controlled by systemd which it may not be, so 
    ###  this should be improved in the future.
    if $enable == true {
        exec { "start-desktop-target":
            onlyif      => "/bin/systemctl status graphical.target |grep inactive",
            command     => "/bin/systemctl isolate graphical.target",
        }
        exec { "enable-desktop-target":
            unless      => "/bin/systemctl get-default |grep graphical",
            command     => "/bin/systemctl set-default graphical.target",
        }
        # Disable some unnecessary services
        # package { ["deja-dup", "zeitgeist-datahub", "zeitgeist-core", "evolution-data-server", "evolution-data-server-common", "evolution-data-server-online-accounts"]
        # Changed mind - if desktop is running then we probably don't care what random processes come with it.  Shut desktop down for flight anyway.
    } elsif $enable == false {
        exec { "stop-desktop-target":
            unless      => "/bin/systemctl status graphical.target |grep inactive",
            command     => "/bin/systemctl isolate multi-user.target",
            timeout     => 500,
        }
        exec { "disable-desktop-target":
            unless      => "/bin/systemctl get-default |grep multi-user;",
            command     => "/bin/systemctl set-default multi-user.target",
        }
    }
    
}