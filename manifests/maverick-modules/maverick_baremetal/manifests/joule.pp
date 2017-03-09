class maverick_baremetal::joule (
    $remove_more_packages = true,
) {
    
    ### Install MRAA - Intel GPIO access library
    class { "apt": }
    apt::ppa { 'ppa:mraa/mraa': 
        notify => Exec['apt_update']
    } ->
    package { ["libmraa1", "libmraa-dev", "mraa-tools", "python-mraa", "python3-mraa"]:
        ensure      => installed,
        require     => Exec['apt_update'],
    }
    
    # eMMC only has 16b, so remove some unnecessary packages if we've started from desktop
    if $remove_more_packages == true {
        package { [
            "fonts-noto-cjk",  "firefox", "thunderbird", "libreoffice-core", "libreoffice-common", "mythes-en-au", "mythes-en-us", "libmythes-1.2-0",
            "ubuntu-docs", "gnome-user-guide", "snapd", "openjdk-8-jre-headless", "openjdk-8-jre", "samba-common", "samba-common-bin", "samba-libs", "ubuntu-online-tour",
            "aisleriot", "gnome-sudoku", "gnome-mahjongg", "gnome-mines", "imagemagick", "imagemagick-6.q16", "imagemagick-common", 
            "cups-browsed", "cups-bsd", "cups-client", "cups-common", "cups-pk-helper", "cups-ppdc", "cups-server-common",
            "shotwell", "shotwell-common", "transmission-common", "transmission-gtk", "libwebkit2gtk-4.0-37-gtk2", "fonts-nanum", 
        ]:
            ensure      => purged
        }
    }
    
    # Install vaapi support
    #package { ["i965-va-driver", "libva-x11-1", "libvdpau-va-gl1", "vainfo", "libva1", "libva-dev", "libva-drm1", "libva-egl1", "libva-glx1", "libva-tpi1", "va-driver-all"]:
    #    ensure      => installed
    #}
    # Install GL support
    #package { ["libegl1-mesa-drivers", "libgles1-mesa", "libosmesa6", "mesa-va-drivers", "libegl1-mesa", "libgl1-mesa-dri", "libgl1-mesa-glx", "libglapi-mesa", "libgles2-mesa"]:
    #    ensure      => installed
    #}
    # Install misc support
    #package { ["intel-gpu-tools", "libcairo2", "libdrm-intel1", "libdrm2"]:
    #    ensure      => installed
    #}
    # Yes this is wierd, but it's needed for intel graphics updater
    #package { ["fonts-ancient-scripts", "ttf-ancient-fonts" ]:
    #    ensure      => installed
    #}
    
}
