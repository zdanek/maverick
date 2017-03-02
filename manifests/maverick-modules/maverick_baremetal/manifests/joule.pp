class maverick_baremetal::joule (
) {
    
    ### Install MRAA - Intel GPIO access library
    class { "apt": }
    apt::ppa { 'ppa:mraa/mraa': 
        notify => Exec['apt_update']
    } ->
    package { ["libmraa1", "libmraa-dev", "mraa-tools", "python-mraa", "python3-mraa"]:
        ensure      => installed
    }
    
    # eMMC only has 16b, so remove some unnecessary packages if we've started from desktop
    package { ["fonts-noto-cjk", "liboxideqtcore0", "firefox", "thunderbird", "libreoffice-core", "libreoffice-common", "ubuntu-docs", "gnome-user-guide", "snapd", "app-install-data", "libwebkit2gtk-4.0-37"]:
        ensure      => purged
    }
    
    # Install vaapi support
    package { ["i965-va-driver", "libva-x11-1", "libvdpau-va-gl1", "vainfo", "libva1", "libva-dev", "libva-drm1", "libva-egl1", "libva-glx1", "libva-tpi1", "va-driver-all"]:
        ensure      => installed
    }
    # Install GL support
    package { ["libegl1-mesa-drivers", "libgles1-mesa", "libosmesa6", "mesa-va-drivers", "libegl1-mesa", "libgl1-mesa-dri", "libgl1-mesa-glx", "libglapi-mesa", "libgles2-mesa"]:
        ensure      => installed
    }
    # Install misc support
    package { ["intel-gpu-tools", "libcairo2", "libdrm-intel1", "libdrm2"]:
        ensure      => installed
    }
    # Yes this is wierd, but it's needed for intel graphics updater
    package { ["fonts-ancient-scripts", "ttf-ancient-fonts" ]:
        ensure      => installed
    }
    
}