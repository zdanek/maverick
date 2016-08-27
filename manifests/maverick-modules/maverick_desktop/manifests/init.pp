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
    } elsif $enable == false {
        exec { "stop-desktop-target":
            unless      => "/bin/systemctl status graphical.target |grep inactive",
            command     => "/bin/systemctl isolate multi-user.target",
        }
        exec { "disable-desktop-target":
            unless      => "/bin/systemctl get-default |grep multi-user;",
            command     => "/bin/systemctl set-default multi-user.target",
        }
    }
    
}