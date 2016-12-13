class maverick_baremetal::odroid::kernel3x {
    
    if $::odroid_kernel3x_backups == "yes" and $::odroid_kernel_current =~ /^4/ {
        warning("Kernel 3.x backup files, restoring 3.x kernel.  A full shutdown and poweroff is necessary before rebooting to restored kernel.")
        exec { "kern3x-bootini":
            command         => "/bin/mv -f /media/boot/boot.ini-k3bak /media/boot/boot.ini",
            onlyif          => "/bin/ls /media/boot/boot.ini-k3bak",
        }
        exec { "kern3x-config":
            command         => "/bin/mv -f /media/boot/config-k3bak /media/boot/config",
            onlyif          => "/bin/ls /media/boot/config-k3bak",
        }
        exec { "kern3x-dtb":
            command         => "/bin/mv -f /media/boot/exynos5422-odroidxu3.dtb-k3bak /media/boot/exynos5422-odroidxu3.dtb",
            onlyif          => "/bin/ls /media/boot/exynos5422-odroidxu3.dtb-k3bak",
        }
        exec { "kern3x-uinit":
            command         => "/bin/mv -f /media/boot/uInitrd-k3bak /media/boot/uInitrd",
            onlyif          => "/bin/ls /media/boot/uInitrd-k3bak",
        }
        exec { "kern3x-zimage":
            command         => "/bin/mv -f /media/boot/zImage-k3bak /media/boot/zImage",
            onlyif          => "/bin/ls /media/boot/zImage-k3bak",
        }
    } elsif $::odroid_kernel_current =~ /^4/ {
        warning("Kernel 3.x backup files don't exist, cannot restore 3.x kernel")
    }
    
}