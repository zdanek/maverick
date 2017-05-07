class maverick_baremetal::joule (
    $remove_more_packages = true,
) {
    
    # Expand rootfs
    if $rootpart_expanded == "False" and $rootpart_device {
        warning("Root Partition does not fill available disk, expanding.  Please reboot after this run.")
        file { "/fsexpand":
            content     => template("maverick_baremetal/fsexpand.erb"),
            mode        => "755",
        } ->
        file { "/.fsexpand":
            ensure      => present,
        }
    }
    
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
            "cups-browsed", "cups-bsd", "cups-client", "cups-common", "cups-ppdc", "cups-server-common",
            "shotwell", "shotwell-common", "transmission-common", "transmission-gtk", "libwebkit2gtk-4.0-37-gtk2", "fonts-nanum", 
        ]:
            ensure      => purged
        }
    }
    # Ensure control center is still installed
    package { "unity-control-center":
        ensure      => installed
    }
    
    # Install vaapi support
    exec { "01org-gfx-repo-key":
        command         => "/usr/bin/wget --no-check-certificate https://download.01.org/gfx/RPM-GPG-KEY-ilg-4 -O - | apt-key add -",
        unless          => "/usr/bin/apt-key list |/bin/grep 39B88DE4",
        notify      => Exec["maverick-aptget-update"],
    } ->
    file { "/etc/apt/sources.list.d/01org-graphics.list":
        content     => "deb https://download.01.org/gfx/ubuntu/16.04/main xenial main",
        notify      => Exec["maverick-aptget-update"],
    } ->
    # Do an explicit apt update here just to be sure    
    exec { "01org-gfx-aptupdate":
        command     => "/usr/bin/apt-get update",
        unless      => "/usr/bin/apt-cache show i915-4.6.3-4.4.0-dkms"
    } ->
    package { ["i915-4.6.3-4.4.0-dkms", "i965-va-driver", "libva-x11-1", "libvdpau-va-gl1", "vainfo", "libva1", "libva-dev", "libva-drm1", "libva-egl1", "libva-glx1", "libva-tpi1", "va-driver-all"]:
        ensure          => latest,
    } ->
    # Install GL support
    package { ["libegl1-mesa-drivers", "libgles1-mesa", "libosmesa6", "mesa-va-drivers", "libegl1-mesa", "libgl1-mesa-dri", "libgl1-mesa-glx", "libglapi-mesa", "libgles2-mesa"]:
        ensure          => latest,
    } ->
    # Install misc support
    package { ["intel-gpu-tools", "libcairo2", "libdrm-intel1", "libdrm2"]:
        ensure          => latest,
    } ->
    # Yes this is wierd, but it's needed for intel graphics updater
    package { ["fonts-ancient-scripts", "ttf-ancient-fonts" ]:
        ensure          => latest,
    }

    ### Install beignet/opencl
    # Install dependencies
    ensure_packages(["cmake", "pkg-config", "ocl-icd-dev", "libegl1-mesa-dev", "ocl-icd-opencl-dev", "libdrm-dev", "libxfixes-dev", "libxext-dev", "llvm-3.6-dev", "clang-3.6", "libclang-3.6-dev", "libtinfo-dev", "libedit-dev", "zlib1g-dev", "clinfo"])
    if ! ("install_flag_beignet" in $installflags) {
        # Clone and Build beignet
        oncevcsrepo { "git-beignet":
            gitsource   => "https://github.com/intel/beignet.git",
            dest        => "/srv/maverick/var/build/beignet",
            revision    => "Release_v1.3.1",
        } ->
        # Create build directory
        file { "/srv/maverick/var/build/beignet/build":
            ensure      => directory,
            owner       => "mav",
            group       => "mav",
            mode        => 755,
        } ->
        file { ["/etc/OpenCL", "/etc/OpenCL/vendors"]:
            ensure      => directory,
            owner       => "mav",
            group       => "mav",
            mode        => "755",
        } ->
        exec { "beignet-prepbuild":
            user        => "mav",
            timeout     => 0,
            command     => "/usr/bin/cmake -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/beignet .. >/srv/maverick/var/log/build/beignet.cmake.out 2>&1",
            cwd         => "/srv/maverick/var/build/beignet/build",
            creates     => "/srv/maverick/var/build/beignet/build/Makefile",
            require     => [ Package["ocl-icd-dev"], Package["libclang-3.6-dev"] ],
        } ->
        exec { "beignet-build":
            user        => "mav",
            timeout     => 0,
            command     => "/usr/bin/make -j${::processorcount} >/srv/maverick/var/log/build/beignet.build.out 2>&1",
            cwd         => "/srv/maverick/var/build/beignet/build",
            creates     => "/srv/maverick/var/build/beignet/build/src/libcl.so",
            require     => Exec["beignet-prepbuild"],
        } ->
        exec { "beignet-install":
            user        => "mav",
            timeout     => 0,
            command     => "/usr/bin/make install >/srv/maverick/var/log/build/beignet.install.out 2>&1",
            cwd         => "/srv/maverick/var/build/beignet/build",
            creates     => "/srv/maverick/software/beignet/lib/beignet/libcl.so",
        } ->
        file { "/srv/maverick/var/build/.install_flag_beignet":
            ensure      => file,
            owner       => "mav",
        }
    }

}
