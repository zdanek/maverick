class maverick-baremetal::raspberry::lcd-adafruit28r (
    ) {
    
    # Add module for touchscreen overlay to boot
    exec { "adafruit28r-stmpmod":
        command     => '/bin/echo "stmpe_ts" >>/etc/modules',
        unless      => '/bin/grep -e "^stmpe_ts" /etc/modules',
    }
    
    # Add udev rule for touchscreen
    file { "/etc/udev/rules.d/95-stmpe.rules":
        content     => 'SUBSYSTEM=="INPUT", ATTRS{name}=="stmpe-ts", ENV{DEVNAME}=="*event*", SYMLINK+="input/touchscreen"',
        owner       => "root",
        mode        => 644
    }
    
    # Setup the pitft28-resistive overlay which seems to magically take care of everything for us
    exec { "adafruit28r-editoverlay":
        command     => '/bin/sed /boot/config.txt -i -r -e "s/^(dtoverlay\=pitft28-resistive)(=[^,]*)?/\dtoverlay=pitft28-resistive,speed=32000000,rotate=90/"',
        onlyif      => "/bin/grep 'dtoverlay\\=pitft28-resistive' /boot/config.txt |/bin/grep -v 'speed=32000000,rotate=90'"
    }
    exec { "adafruit28r-addoverlay":
        command     => "/bin/echo 'dtoverlay=pitft28-resistive,speed=32000000,rotate=90' >>/boot/config.txt",
        unless      => "/bin/grep 'dtoverlay\\=pitft28-resistive' /boot/config.txt"
    }
    
    # Redirect the console to the touchscreen
    exec { "adafruit28r-editfbcon":
        command     => '/bin/sed /boot/cmdline.txt -i -r -e "s/fbcon\=map:([0-9]+)/fbcon=map:1/"',
        onlyif      => "/bin/grep 'fbcon\\=map:' /boot/cmdline.txt |/bin/grep -v 'fbcon\\=map:1'"
    }
    exec { "adafruit28r-addfbcon":
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
    exec { "adafruit28r-conblank":
        command     => '/bin/sed /etc/kbd/config -i -r -e "s/^BLANK_TIME\\=30/BLANK_TIME=0/"',
        unless      => "/bin/grep 'BLANK_TIME\\=0' /etc/kbd/config",
    }
    
    # Install fbcp (framebuffer copying) to output GPU accelerated graphics to TFT screen
    class { "maverick-baremetal::raspberry::fbcp": }
        
    # Alter console hdmi resolution to match the LCD, so framebuffer copy doesn't have to scale
    class { "maverick-baremetal::raspberry::console":
        width       => 320,
        height      => 240,
    }
    
}
