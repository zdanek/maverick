class maverick-baremetal::raspberry::init (
    $expand_root = true,
    $gpumem = 256,
    $overclock = "None", # "None", "High", "Turbo"
    $devicetree = true,
    $spi = false,
    $i2c = false,
    $serialconsole = false, # Normally leave the serial lines free for pixhawk
    $camera = true,
    $xgl = false,
    ) {
        
    if ($expand_root) {
        exec { "raspberry-expandroot":
            command     => "/usr/bin/raspi-config nonint do_expand_rootfs; echo 'done' > /etc/raspi-expandroot",
            unless     => "/bin/grep 'done' /etc/raspi-expandroot",
        }
    }
        
    exec { "raspberry-setgpumem":
        command     => "/usr/bin/raspi-config nonint do_memory_split ${gpumem}",
        unless      => "/bin/grep 'gpu_mem=${gpumem}' /boot/config.txt"
    }
    
    if ($overclock) {
        exec { "raspberry-overlock":
            command     => "/usr/bin/raspi-config nonint do_overclock ${overclock}; echo '${overclock}' >/etc/raspi-overclock",
            unless      => "/bin/grep '${overclock}' /etc/raspi-overclock"
        }
    }
    
    if ($devicetree == true) {
        exec { "raspberry-devicetree":
            command     => "/usr/bin/raspi-config nonint do_devicetree 0",
            onlyif	=> "/bin/grep 'device_tree' /boot/config.txt |/bin/grep -v '#device_tree'"
        }
    } else {
        exec { "raspberry-devicetree":
            command     => "/usr/bin/raspi-config nonint do_devicetree 1",
            unless      => "/bin/grep 'device_tree' /boot/config.txt |/bin/grep -v '#device_tree'"
        }
    }

    if ($spi == true) {
        exec { "raspberry-spi":
            command     => "/usr/bin/raspi-config nonint do_spi 0",
            unless      => "/bin/grep '^dtparam=spi=on' /boot/config.txt"
        }
    } else {
        exec { "raspberry-spi":
            command     => "/usr/bin/raspi-config nonint do_spi 1",
            unless      => "/bin/grep '^dtparam=spi=off' /boot/config.txt"
        }
    }
    
    if ($i2c == true) {
        exec { "raspberry-i2c":
            command     => "/usr/bin/raspi-config nonint do_i2c 0",
            unless      => "/bin/grep '^dtparam=i2c_arm=on' /boot/config.txt"
        }
    } else {
        exec { "raspberry-i2c":
            command     => "/usr/bin/raspi-config nonint do_i2c 1",
            unless      => "/bin/grep '^dtparam=i2c_arm=off' /boot/config.txt"
        }
    }

    if ($serialconsole == false) {
        exec { "raspberry-serial":
            command     => "/usr/bin/raspi-config nonint do_serial 1",
            onlyif      => "/bin/grep 'console=serial' /boot/cmdline.txt"
        }
    } else {
        exec { "raspberry-serial":
            command     => "/usr/bin/raspi-config nonint do_serial 0",
            unless      => "/bin/grep 'console=serial' /boot/cmdline.txt"
        }
    }
    
    if ($camera == true) {
        exec { "raspberry-camera":
            command     => "/usr/bin/raspi-config nonint set_camera 1; echo 'true' >/etc/raspi-camera",
            unless      => "/bin/grep 'true' /etc/raspi-camera",
        }
    } else {
        exec { "raspberry-camera":
            command     => "/usr/bin/raspi-config nonint set_camera 0; echo 'false' >/etc/raspi-camera",
            unless      => "/bin/grep 'false' /etc/raspi-camera",
        }
    }
    
    if ($xgl == true) {
        package { ["libgl1-mesa-dri", "xcompmgr" ]:
            ensure      => present
        } ->
        exec { "raspberry-xgl":
            command     => "/usr/bin/raspi-config nonint do_gldriver 1; echo 'true' >/etc/raspi-xgl",
            unless      => "/bin/grep 'true' /etc/raspi-xgl",
        }
    } else {
        exec { "raspberry-xgl":
            command     => "/usr/bin/raspi-config nonint do_gldriver 0; echo 'false' >/etc/raspi-xgl",
            unless      => "/bin/grep 'false' /etc/raspi-xgl",
        }
    }
    
    exec { "raspberry-disableconfig":
        command     => "/usr/bin/raspi-config nonint disable_raspi_config_at_boot",
        onlyif      => "/bin/ls /etc/profile.d/raspi-config.sh"
    }
    
    exec { "raspberry-bootenv":
        command     => "/usr/bin/raspi-config nonint do_boot_behaviour_new B1",
        unless      => "/bin/systemctl get-default |/bin/grep multi-user"
    }
    
}
