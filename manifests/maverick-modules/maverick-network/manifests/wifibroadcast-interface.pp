define maverick-network::wifibroadcast-interface (
    $macaddress = undef,
) {

    # If the mac address is specified, then set the interface name statically in udev
	if $macaddress {
    	concat::fragment { "interface-customname-${name}":
            target      => "/etc/udev/rules.d/10-network-customnames.rules",
            content     => "SUBSYSTEM==\"net\", ACTION==\"add\", ATTR{address}==\"${macaddress}\", NAME=\"${name}\"\n",
        }
	}
	
    # Define a service definition for this interface
	service { "wifibroadcast@${name}":
        ensure      => running,
        enable      => true,
        require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/wifibroadcast@.service"] ]
    }

    # Add a config file for this interface
    file { "/srv/maverick/data/config/wifibroadcast-${name}.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        replace     => false, # initialize but don't overwrite in the future
        source      => "puppet:///modules/maverick-network/wifibroadcast-if.conf",
    }

}