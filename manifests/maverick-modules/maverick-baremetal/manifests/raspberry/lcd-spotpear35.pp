class maverick-baremetal::raspberry::lcd-spotpear35 (
    ) {
    
    # This used to be very painful, now all of the necessary kernel support is in raspbian
    # Amongst other sources, these were useful:
    #  http://futurice.com/blog/id-like-to-have-some-lcd-on-my-pi
    #  https://github.com/notro/fbtft/issues/215
    # The recipe here is for non-X console framebuffer in fixed landscape mode, with uncalibrated touchscreen support.
    # We may add calibrated X mode later on.
    
    # Setup the correct kernel module parameters
    #augeas { "spotpear35-flexfbmod":
    #    onlyif  => "get /files/etc/modules/flexfb != 'regwidth=16 nobacklight init=-1,0xb0,0x0,-1,0x11,-2,250,-1,0x3A,0x55,-1,0xC2,0x44,-1,0xC5,0x00,0x00,0x00,0x00,-1,0xE0,0x0F,0x1F,0x1C,0x0C,0x0F,0x08,0x48,0x98,0x37,0x0A,0x13,0x04,0x11,0x0D,0x00,-1,0xE1,0x0F,0x32,0x2E,0x0B,0x0D,0x05,0x47,0x75,0x37,0x06,0x10,0x03,0x24,0x20,0x00,-1,0xE2,0x0F,0x32,0x2E,0x0B,0x0D,0x05,0x47,0x75,0x37,0x06,0x10,0x03,0x24,0x20,0x00,-1,0x36,0x28,-1,0x11,-1,0x29,-3 width=480 height=320'",
    #    changes => "set /files/etc/modules/flexfb 'regwidth=16 nobacklight init=-1,0xb0,0x0,-1,0x11,-2,250,-1,0x3A,0x55,-1,0xC2,0x44,-1,0xC5,0x00,0x00,0x00,0x00,-1,0xE0,0x0F,0x1F,0x1C,0x0C,0x0F,0x08,0x48,0x98,0x37,0x0A,0x13,0x04,0x11,0x0D,0x00,-1,0xE1,0x0F,0x32,0x2E,0x0B,0x0D,0x05,0x47,0x75,0x37,0x06,0x10,0x03,0x24,0x20,0x00,-1,0xE2,0x0F,0x32,0x2E,0x0B,0x0D,0x05,0x47,0x75,0x37,0x06,0x10,0x03,0x24,0x20,0x00,-1,0x36,0x28,-1,0x11,-1,0x29,-3 width=480 height=320'",
    #} ->
    #augeas { "spotpear35-fbtftmod":
    #    onlyif      => "get /files/etc/modules/fbtft_device != 'name=flexfb speed=16000000 gpios=reset:25,dc:24'",
    #    changes     => "set /files/etc/modules/fbtft_device 'name=flexfb speed=16000000 gpios=reset:25,dc:24'",
    #}
    
    # Setup the touchscreen for landscape orientation
    #exec { "spotpear35-sedtouch":
    #    command     => '/bin/sed /boot/config.txt -i -r -e "s/^(dtoverlay\=ads7846)(=[^,]*)?/\dtoverlay=ads7846,speed=500000,penirq=17,swapxy=1/"',
    #    onlyif      => "/bin/grep 'dtoverlay\\=ads7846' /boot/config.txt |/bin/grep -v 'speed=500000,penirq=17,swapxy=1'"
    #}
    #exec { "spotpear35-addtouch":
    #    command     => "/bin/echo 'dtoverlay=ads7846,speed=500000,penirq=17,swapxy=1' >>/boot/config.txt",
    #    unless      => "/bin/grep 'dtoverlay\\=ads7846' /boot/config.txt"
    #}

    # Setup the piscreen overlay which seems to magically take care of everything for us
    exec { "spotpear35-editoverlay":
        command     => '/bin/sed /boot/config.txt -i -r -e "s/^(dtoverlay\=piscreen)(=[^,]*)?/\dtoverlay=piscreen,speed=16000000,rotate=90/"',
        onlyif      => "/bin/grep 'dtoverlay\\=piscreen' /boot/config.txt |/bin/grep -v 'speed=16000000,rotate=90'"
    }
    exec { "spotpear35-addoverlay":
        command     => "/bin/echo 'dtoverlay=piscreen,speed=16000000,rotate=90' >>/boot/config.txt",
        unless      => "/bin/grep 'dtoverlay\\=piscreen' /boot/config.txt"
    }
    
    # Redirect the console to the touchscreen
    exec { "spotpear35-editfbcon":
        command     => '/bin/sed /boot/cmdline.txt -i -r -e "s/fbcon\=map:([0-9]+)/fbcon=map:1/"',
        onlyif      => "/bin/grep 'fbcon\\=map:' /boot/cmdline.txt |/bin/grep -v 'fbcon\\=map:1'"
    }
    exec { "spotpear35-addfbcon":
	command     => '/bin/sed /boot/cmdline.txt -i -r -e "s/$/ fbcon=map:1/"',
	unless	    => "/bin/grep 'fbcon\\=map:' /boot/cmdline.txt",
    }
    
    # Set console font
    file { "/etc/default/console-setup":
	source      => 'puppet:///modules/maverick-baremetal/console-setup',
	mode        => 644,
        owner       => 'root',
        group       => 'root',
    }
 
    # Turn off console blanking
    exec { "spotpear35-conblank":
        command     => '/bin/sed /etc/kbd/config -i -r -e "s/^BLANK_TIME\\=30/BLANK_TIME=0/"',
        unless      => "/bin/grep 'BLANK_TIME\\=0' /etc/kbd/config",
    }
    
    # Install fbcp (framebuffer copying) to output GPU accelerated graphics to TFT screen
    class { "maverick-baremetal::raspberry::fbcp": }
        
    # Alter console hdmi resolution to match the LCD, so framebuffer copy doesn't have to scale
    class { "maverick-baremetal::raspberry::console":
        width       => 320,
        height      => 480,
    }
    

}
