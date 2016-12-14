class maverick_baremetal::odroid::kernel4x (
) {
    
    # Backup existing kernel files
    if $::odroid_kernel3x_backups != "yes" and $::odroid_kernel_current !~ /^4/ {
        exec { "odroid-backup-kernel3":
            cwd         => "/media/boot",
            command     => "/bin/bash -c 'for f in *; do cp \$f \$f-k3bak; done'",
            creates     => "/media/boot/boot.ini-k3bak",
        }
    }
    
    # Freeze installed kernels
    exec { "odroid-freeze-kernels":
        command     => "/usr/bin/apt-mark hold linux-image-xu3",
        unless      => "/usr/bin/dpkg -l linux-image-xu3 |tail -1 |grep ^h"
    }
    
    # Ensure dependencies are installed
    ensure_packages(["bc", "curl", "gcc", "libncurses5-dev", "lzop", "make", "u-boot-tools", "dos2unix"])
    
    # Clone and build 4.x kernel for xu4
    oncevcsrepo { "git-odroid-kernel4x":
        gitsource   => "https://github.com/Dmole/linux.git",
        # revision    => "odroidxu4-mihailescu2m-4.8",
        revision    => "odroidxu4-mihailescu2m-4.9.y",
        dest        => "/srv/maverick/var/build/linux",
        owner       => "mav",
        group       => "mav",
    } ->
    #exec { "odroid-kernel4x-makedep":
    #    timeout     => 0,
    #    command     => "/usr/bin/make odroidxu4_defconfig >/srv/maverick/var/log/build/odroid-kernel4x.makedep.log 2>&1",
    #    cwd         => "/srv/maverick/var/build/linux",
    #    creates     => "/srv/maverick/var/build/linux/.config",
    #} ->
    file { "/srv/maverick/var/build/linux/.config": # use a predefined .config instead of creating one from recipe
        source      => "puppet:///modules/maverick_baremetal/odroidxu4-kern4x-config",
    } ->
    exec { "odroid-kernel4x-make":
        timeout     => 0,
        command     => "/usr/bin/make -j 8 zImage dtbs modules >/srv/maverick/var/log/build/odroid-kernel4x.make.log 2>&1",
        cwd         => "/srv/maverick/var/build/linux",
        # creates     => "/srv/maverick/var/build/linux/arch/arm/boot/zImage",
        require     => [ Package["bc"], Package["make"] ],
        user        => "mav",
    } ->
    exec { "odroid-kernel4x-marker":
        command     => "/usr/bin/make -C /srv/maverick/var/build/linux -s kernelrelease >/srv/maverick/var/build/linux/.kernelrelease",
        # creates     => "/srv/maverick/var/build/linux/.kernelrelease",
        user        => "mav",
    }
    
    # Get the version of the kernel we're building.  Note that we use generate here which runs on the 'master'.  If
    #  maverick is not running as a puppet agent (ie master/client model), this will break.
    if $::odroid_kernel4x_release != "no" {
        $kver = $::odroid_kernel4x_release
        warning("Building second stage Odroid 4.x kernel, please reboot once this run is complete.")
    } else {
        $kver = undef
        warning("Building Odroid 4.x kernel is currently a two stage process, please run maverick --configure again after this run is complete.")
    }
    
    if $kver {
        exec { "odroid-kernel4x-fw":
            command     => "/usr/bin/make firmware_install; /usr/bin/touch /lib/firmware/.fw-${kver}",
            cwd         => "/srv/maverick/var/build/linux",
            # creates     => "/lib/firmware/.fw-${kver}"
            require     => Exec["odroid-kernel4x-make"]
        } ->
        exec { "odroid-kernel4x-mods":
            command     => "/usr/bin/make INSTALL_MOD_STRIP=1 modules_install",
            cwd         => "/srv/maverick/var/build/linux",
            # creates     => "/lib/modules/${kver}/modules.dep",
            require     => Exec["odroid-kernel4x-make"]
        } ->
        exec { "odroid-kernel4x-cpkern":
            command     => "/bin/cp -f arch/arm/boot/zImage arch/arm/boot/dts/exynos5422-odroidxu[34].dtb /media/boot",
            cwd         => "/srv/maverick/var/build/linux",
            # creates     => "/media/boot/exynos5422-odroidxu4.dtb",
        } ->
        exec { "odroid-kernel4x-config":
            command     => "/bin/cp -f .config /boot/config-${kver}",
            cwd         => "/srv/maverick/var/build/linux",
            # creates     => "/boot/config-${kver}",
        } ->
        exec { "odroid-kernel4x-initramfs":
            command     => "/usr/sbin/update-initramfs -c -k ${kver}",
            cwd         => "/srv/maverick/var/build/linux",
            # creates     => "/boot/initrd.img-${kver}"
        } ->
        exec { "odroid-kernel4x-mkimage":
            command     => "/usr/bin/mkimage -A arm -O linux -T ramdisk -a 0x0 -e 0x0 -n initrd.img-${kver} -d initrd.img-${kver} uInitrd-${kver}",
            cwd         => "/boot",
            # creates     => "/boot/uInitrd-${kver}",
        } ->
        exec { "odroid-kernel4x-cpuinit":
            command     => "/bin/cp -f /boot/uInitrd-${kver} /media/boot/uInitrd",
            # unless      => "/usr/bin/mkimage -l /media/boot/uInitrd |/bin/grep Name |/bin/grep initrd.img-${kver}"
        } ->
        exec { "odroid-kernel4x-xu4ini":
            command     => "/bin/sed -i -e 's/odroidxu3/odroidxu4/' /media/boot/boot.ini",
            # onlyif      => "/bin/grep odroidxu3 /media/boot/boot.ini"
        } ->
        file { "/srv/maverick/var/build/linux/.install_flag":
            ensure      => present,
        }
                
    }
    
    
}