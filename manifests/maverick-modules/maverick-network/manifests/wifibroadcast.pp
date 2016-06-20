define maverick-network::wifibroadcast (
    $macaddress = undef,
) {

    # If the mac address is specified, then set the interface name statically in udev
	if $macaddress {
    	concat::fragment { "interface-customname-${name}":
            target      => "/etc/udev/rules.d/10-network-customnames.rules",
            content     => "SUBSYSTEM==\"net\", ACTION==\"add\", ATTR{address}==\"${macaddress}\", NAME=\"${name}\"\n",
        }
	}
    
}