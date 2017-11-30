class maverick_hardware::joule (
    $remove_more_packages = true,
    $ipu4_blacklist = true,
    $serialconsole = false,
    $install_caspa = true,
) {

    # Include Intel platform manifest
    class { "maverick_hardware::intel": }
    
    # Control serial console, disable by default to clear the way for mavlink connection
    if $serialconsole == false {
        service_wrapper {"serial-getty@ttyS1":
            ensure  => stopped,
            enable  => false
        }
    } else {
        service_wrapper { "serial-getty@ttyS1":
            ensure  => running,
            enable  => true
        }
    }
    
    # Expand rootfs
    if str2bool($::rootpart_expanded) == false and str2bool($::rootpart_device) and str2bool($::rootpart_partition) and str2bool($::rootpart_partno) {
        warning("Root Partition does not fill available disk, expanding.  Please reboot after this run.")
        file { "/fsexpand":
            content     => template("maverick_hardware/fsexpand.erb"),
            mode        => "755",
        } ->
        file { "/.fsexpand":
            ensure      => present,
        } ->
        file { "/etc/rc.local":
            source      => "puppet:///modules/maverick_hardware/rc.local",
            mode        => "755",
            owner       => "root",
            group       => "root",
        }
    }
    
    # 550 eMMC only has 8Gb, so remove some unnecessary packages if we've started from desktop
    if $remove_more_packages == true {
        package { [
            "fonts-noto-cjk",  "firefox", "thunderbird", "mythes-en-au", "mythes-en-us", "libmythes-1.2-0",
            "ubuntu-docs", "gnome-user-guide", "snapd", "samba-common", "samba-common-bin", "samba-libs", "ubuntu-online-tour",
            "aisleriot", "gnome-sudoku", "gnome-mahjongg", "gnome-mines", "imagemagick", "imagemagick-6.q16", "imagemagick-common", 
            "cups-browsed", "cups-bsd", "cups-client", "cups-common", "cups-ppdc", "cups-server-common",
            "shotwell", "shotwell-common", "transmission-common", "transmission-gtk", "libwebkit2gtk-4.0-37-gtk2", "fonts-nanum", 
        ]:
            ensure      => purged
        }
        package { ["chromium-browser", "liboxideqtcore0", "pepperflashplugin-nonfree", "example-content", "ubuntu-touch-sounds", "fonts-lato", "hplip-data"]:
            ensure      => purged
        }
        if ! getvar("maverick_intelligence::tensorflow") {
            package { ["openjdk-8-jre-headless", "openjdk-8-jre"]:
                ensure      => purged
            }
        }
    }
    # Ensure control center is still installed
    package { "unity-control-center":
        ensure      => installed
    }

    # Blacklist the ipu4 stuff, it's buggy and messy
    if $ipu4_blacklist == true {
        file { "/etc/modprobe.d/blacklist-ipu4.conf":
            ensure      => file,
            source      => "puppet:///modules/maverick_hardware/blacklist-ipu4.conf",
            mode        => "644",
            owner       => "root",
            group       => "root",
        }
    } else {
        file { "/etc/modprobe.d/blacklist-ipu4.conf":
            ensure      => absent,
        }
    }

    if $install_caspa == true {
        class { "maverick_hardware::peripheral::caspa": }
    }
    
}
