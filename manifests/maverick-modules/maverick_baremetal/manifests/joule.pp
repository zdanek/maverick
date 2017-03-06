class maverick_baremetal::joule (
    $remove_more_packages = false,
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
            "fonts-noto-cjk",  "firefox", "thunderbird", "libreoffice-core", "libreoffice-common", "ubuntu-docs", "gnome-user-guide", "snapd", "app-install-data", "mythes-en-au", "mythes-en-us", "libmythes-1.2-0",
            "chromium-browser", "openjdk-8-jre-headless", "libicu-dev", "liboxideqtcore0", "libboost1.58-dev", "sbcl", "python-samba", "samba-common", "samba-common-bin", "samba-libs",
            "gfortran-5",
            "adwaita-icon-theme", "hicolor-icon-theme", "humanity-icon-theme", "notify-osd-icons", "suru-icon-theme", "ubuntu-mobile-icons", "ubuntu-wallpapers", "ubuntu-wallpapers-xenial", "example-content",
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
