class maverick_desktop (
    $enable = false,
    $desktop_suspend = false,
) {
    
    ### Desktop is disabled by default and must be specifically enabled
    ### This assumes that the desktop is controlled by systemd which it may not be, so 
    ###  this should be improved in the future.
    if $enable == true {
        
        # If raspberry platform, ensure pixel desktop is installed
        if $raspberry_present == "yes" {
            ensure_packages(["xserver-xorg", "xinit", "raspberrypi-ui-mods", "lightdm"], {'before'=>Exec["start-desktop-target"]})
        }
        
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

        # Enable grub splash, which ensures graphical tty7 is displayed by default
        exec { "grub-splash":
            command     => "/bin/sed -i -e 's/GRUB_CMDLINE_LINUX_DEFAULT=\"\"/GRUB_CMDLINE_LINUX_DEFAULT=\"quiet splash\"/' /etc/default/grub; /usr/sbin/update-grub",
            onlyif      => "/bin/grep 'GRUB_CMDLINE_LINUX_DEFAULT=\"\"' /etc/default/grub",
        }
        
        # Tegra graphical/multi-user systemd target is broken
        if $tegra_present == "yes" {
            service { "lightdm":
                ensures     => running,
                enable      => true,
            }
        }
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

        # Tegra graphical/multi-user systemd target is broken
        if $tegra_present == "yes" {
            service { "lightdm":
                ensure      => stopped,
                enable      => false,
            }
        }

        # Disable grub splash, which ensures tty1 is displayed by default on console at boot, otherwise we get a blank screen
        exec { "grub-splash":
            command     => "/bin/sed -i -e 's/GRUB_CMDLINE_LINUX_DEFAULT=\"quiet splash\"/GRUB_CMDLINE_LINUX_DEFAULT=\"\"/' /etc/default/grub; /usr/sbin/update-grub; /bin/chvt 1",
            onlyif      => "/bin/grep 'GRUB_CMDLINE_LINUX_DEFAULT=\"quiet splash\"' /etc/default/grub",
        }
    }
        
    /*
    # Disable suspend for mav user
    if $enable == true and $desktop_suspend == false {
        exec { "mav_suspend":
            command     => "/usr/bin/dbus-launch gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-timeout 0",
            unless      => "/usr/bin/gsettings get org.gnome.settings-daemon.plugins.power sleep-inactive-ac-timeout |grep -e '^0$'",
            user        => "mav",
        }
    }
    */
    
}