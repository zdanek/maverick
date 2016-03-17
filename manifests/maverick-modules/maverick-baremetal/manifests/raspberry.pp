class maverick-baremetal::raspberry (
    $expand_root = true,
    $gpumem = 256,
    $overclock = "None", # "None", "High", "Turbo"
    $devicetree = true,
    $spi = true,
    $i2c = true,
    $serialconsole = false, # Normally leave the serial lines free for dronekit
    ) {
        
    if ($expand_root) {
        exec { "raspberry-expandroot":
            command     => "/usr/bin/raspi-config nonint do_expand_rootfs ",
            creates     => "/etc/init.d/resize2fs_once",
        }
    }
        
    exec { "raspberry-setgpumem":
        command     => "/usr/bin/raspi-config nonint do_memory_split ${gpumem}",
        unless      => "/bin/grep 'gpu_mem=${gpumem}' /boot/config.txt"
    }
    
    if ($overclock) {
        exec { "raspberry-overlock":
            command     => "/usr/bin/raspi-config nonint do_overclock ${overclock}",
            unless      => "/bin/grep 'over_voltage' /boot/config.txt |/bin/grep -v '#over_voltage'"
        }
    }
    
    if ($devicetree) {
        exec { "raspberry-devicetree":
            command     => "/usr/bin/raspi-config nonint do_devicetree 0",
            unless      => "/bin/grep 'device_tree' /boot/config.txt |/bin/grep -v '#device_tree'"
        }
    }

    if ($spi) {
        exec { "raspberry-spi":
            command     => "/usr/bin/raspi-config nonint do_spi 0",
            unless      => "/bin/grep 'dtparam=spi' /boot/config.txt'"
        }
    }
    
    if ($i2c) {
        exec { "raspberry-i2c":
            command     => "/usr/bin/raspi-config nonint do_i2c 0",
            unless      => "/bin/grep 'dtparam=i2c_arm' /boot/config.txt'"
        }
    }

    if ($serialconsole == false) {
        exec { "raspberry-serial":
            command     => "/usr/bin/raspi-config nonint do_serial 1",
            unless      => "/bin/grep 'console=ttyAMA0' /boot/cmdline.txt'"
        }
    } else {
        exec { "raspberry-serial":
            command     => "/usr/bin/raspi-config nonint do_serial 0",
            unless      => "/bin/grep 'console=serial' /boot/cmdline.txt'"
        }
    }
    
    exec { "raspberry-disableconfig":
        command     => "/usr/bin/raspi-config nonint disable_raspi_config_at_boot_i2c",
        unless      => "/bin/ls /etc/profile.d/raspi-config.sh"
    }
    
    exec { "raspberry-bootenv":
        command     => "/usr/bin/raspi-config nonint do_boot_behaviour_new B1",
        unless      => "/bin/systemctl get-default |/bin/grep multi-user"
    }
    
}