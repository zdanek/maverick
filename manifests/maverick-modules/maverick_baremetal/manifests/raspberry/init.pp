class maverick_baremetal::raspberry::init (
    $expand_root = true,
    $gpumem = 128,
    $overclock = "None", # "None", "High", "Turbo"
    $devicetree = true,
    $spi = true,
    $i2c = true,
    $serialconsole = false, # Normally leave the serial lines free for pixhawk
    $camera = true,
    $xgl = false,
    $v4l2 = true,
    $overpower_usb = false,
    ) {

    ensure_packages(["raspi-config", "python-rpi.gpio", "python3-rpi.gpio", "rpi-update", "raspi-gpio"])
    # Install wiringpi through pip as it's not always available through apt
    python::pip { 'pip-wiringpi':
        pkgname     => 'wiringpi',
        ensure      => present,
    }
    
    if ($expand_root) {
        # $rootpart_expanded and $rootfs_expanded are facts provided by facts.d/filesystems.py
        if $::rootpart_expanded == False or $::rootfs_expanded == False {
            exec { "raspberry-expandroot":
                command     => "/usr/bin/raspi-config nonint do_expand_rootfs; echo 'done' > /etc/raspi-expandroot",
                unless      => "/bin/grep 'done' /etc/raspi-expandroot",
                require	    => Package["raspi-config"],
            }
        }
    }
        
    exec { "raspberry-setgpumem":
        command     => "/usr/bin/raspi-config nonint do_memory_split ${gpumem}",
        unless      => "/bin/grep 'gpu_mem=${gpumem}' /boot/config.txt",
        require     => Package["raspi-config"]
    }
    
    confval{ "rpi-enableuart":
        file        => "/boot/config.txt",
        field       => "enable_uart",
        value       => 1,
    }
    # If this is a RPi 3, disable the onboard bluetooth and reassign the GPIO UART to uart0 which is more reliable
    if ($raspberry_model == "3 Model B") {
        confline { "rpi3-uartoverlay":
            file        => "/boot/config.txt",
            line        => "dtoverlay=pi3-disable-bt",
        }
        service { "hciuart":
            enable      => false,
            ensure      => stopped,
        } ->
        package { "hciuart":
            ensure      => purged
        }
    }

    if ($overclock) {
        exec { "raspberry-overclock":
            command     => "/usr/bin/raspi-config nonint do_overclock ${overclock}; echo '${overclock}' >/etc/raspi-overclock",
            unless      => "/bin/grep '${overclock}' /etc/raspi-overclock",
            require     => Package["raspi-config"],
        }
    }
    
    if ($devicetree == true) {
        exec { "raspberry-devicetree":
            command     => "/usr/bin/raspi-config nonint do_devicetree 0",
            onlyif	    => "/bin/grep 'device_tree' /boot/config.txt |/bin/grep -v '#.*device_tree'",
            require     => Package["raspi-config"],
        }
    } else {
        exec { "raspberry-devicetree":
            command     => "/usr/bin/raspi-config nonint do_devicetree 1",
            unless      => "/bin/grep 'device_tree' /boot/config.txt |/bin/grep -v '#.*device_tree'",
            require     => Package["raspi-config"],
        }
    }

    if ($spi == true) {
        exec { "raspberry-spi":
            command     => "/usr/bin/raspi-config nonint do_spi 0",
            unless      => "/bin/grep '^dtparam=spi=on' /boot/config.txt",
            require     => Package["raspi-config"],
        }
    } else {
        exec { "raspberry-spi":
            command     => "/usr/bin/raspi-config nonint do_spi 1",
            unless      => "/bin/grep '^dtparam=spi=off' /boot/config.txt",
            require     => Package["raspi-config"],
        }
    }
    
    if ($i2c == true) {
        exec { "raspberry-i2c":
            command     => "/usr/bin/raspi-config nonint do_i2c 0",
            unless      => "/bin/grep -E '^dtparam=i2c(_arm)=on' /boot/config.txt",
            require     => Package["raspi-config"],
        }
    } else {
        exec { "raspberry-i2c":
            command     => "/usr/bin/raspi-config nonint do_i2c 1",
            unless      => "/bin/grep -E '^dtparam=i2c(_arm)=off' /boot/config.txt",
            require     => Package["raspi-config"],
        }
    }

    if ($serialconsole == false) {
        exec { "raspberry-serial":
            command     => "/usr/bin/raspi-config nonint do_serial 1",
            onlyif      => "/bin/grep 'console=serial' /boot/cmdline.txt",
            require     => Package["raspi-config"],
        }
    } else {
        exec { "raspberry-serial":
            command     => "/usr/bin/raspi-config nonint do_serial 0",
            unless      => "/bin/grep 'console=serial' /boot/cmdline.txt",
            require     => Package["raspi-config"],
        }
    }
    
    if ($camera == true) {
        exec { "raspberry-camera":
            command     => "/usr/bin/raspi-config nonint set_camera 1; echo 'true' >/etc/raspi-camera",
            unless      => "/bin/grep 'true' /etc/raspi-camera",
            require     => Package["raspi-config"],
        }
    } else {
        exec { "raspberry-camera":
            command     => "/usr/bin/raspi-config nonint set_camera 0; echo 'false' >/etc/raspi-camera",
            unless      => "/bin/grep 'false' /etc/raspi-camera",
            require     => Package["raspi-config"],
        }
    }
    
    if ($xgl == true) {
        package { ["libgl1-mesa-dri", "xcompmgr" ]:
            ensure      => present
        } ->
        exec { "raspberry-xgl":
            command     => "/usr/bin/raspi-config nonint do_gldriver 1; echo 'true' >/etc/raspi-xgl",
            unless      => "/bin/grep 'true' /etc/raspi-xgl",
            require     => Package["raspi-config"],
        }
    } else {
        exec { "raspberry-xgl":
            command     => "/usr/bin/raspi-config nonint do_gldriver 0; echo 'false' >/etc/raspi-xgl",
            unless      => "/bin/grep 'false' /etc/raspi-xgl",
            require     => Package["raspi-config"],
        }
    }
    
    exec { "raspberry-disableconfig":
        command     => "/usr/bin/raspi-config nonint disable_raspi_config_at_boot",
        onlyif      => "/bin/ls /etc/profile.d/raspi-config.sh",
        require     => Package["raspi-config"],
    }
    
    # This is disabled and handled by other classes such as maverick_desktop instead
    #exec { "raspberry-bootenv":
    #    command     => "/usr/bin/raspi-config nonint do_boot_behaviour_new B1",
    #    unless      => "/bin/systemctl get-default |/bin/grep multi-user",
    #    require     => Package["raspi-config"],
    #}
    
    if $v4l2 == true {
        exec { "raspberry-v4l2-kernelmodule":
            command     => "/bin/echo 'bcm2835-v4l2' >>/etc/modules",
            unless      => "/bin/grep '^bcm2835-v4l2' /etc/modules",
        }
        file { "/etc/modprobe.d/raspicam.conf":
            content     => "options bcm2835_v4l2 gst_v4l2src_is_broken=1"
        }
    }
    
    if ($overpower_usb == true) {
        exec { "overpower_usb_true_add":
            command     => "/bin/echo 'max_usb_current=1' >>/boot/config.txt",
            unless      => "/bin/grep -e '^max_usb_current' /boot/config.txt",
        } ->
        exec { "overpower_usb_true":
            command     => "/bin/sed /boot/config.txt -i -r -e 's/^max_usb_current=.*/max_usb_current=1/'",
            unless      => "/bin/grep -e '^max_usb_current=1' /boot/config.txt",
        }
    } else {
        exec { "overpower_usb_false":
            command     => "/bin/sed /boot/config.txt -i -r -e 's/^max_usb_current=1/#max_usb_current=1/'",
            onlyif      => "/bin/grep -e '^max_usb_current=1' /boot/config.txt",
        }
    }
}
