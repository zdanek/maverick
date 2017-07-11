define maverick_network::process_interface (
    $mode = "managed",
    $type = "ethernet",
    $addressing = "dhcp",
    $macaddress = undef,
    $ipaddress = undef,
    $apaddress = "192.168.10.1",
    $gateway = undef,
    $nameservers = undef,
    $ssid = undef,
    $psk = undef,
    $driver  = "nl80211",
    $channel = undef,
    $hw_mode = "g",
    $disable_broadcast_ssid = false,
    $dhcp_range = "192.168.10.10,192.168.10.50",
    $dhcp_leasetime = "24h",
) {
	# Display a warning to reboot if networking config has chagned significantly
    if getvar("::netinfo_${name}_macaddress") != $macaddress {
    	crit("WARNING: Interface config has changed significantly, PLEASE REBOOT TO ACTIVATE")
    }
	
    # If the mac address is specified, then set the interface name statically in udev
	if $macaddress {
    	concat::fragment { "interface-customname-${name}":
            target      => "/etc/udev/rules.d/10-network-customnames.rules",
            content     => "SUBSYSTEM==\"net\", ACTION==\"add\", ATTR{address}==\"${macaddress}\", NAME=\"${name}\"\n",
        }
	}
    # Process managed interfaces
	if $mode == "managed" {
	    maverick_network::interface_managed { $name:
	        type        => $type,
	        addressing  => $addressing,
	        macaddress  => $macaddress,
	        ipaddress   => $ipaddress,
	        gateway     => $gateway,
	        nameservers => $nameservers,
	        ssid        => $ssid,
	        psk         => $psk,
	    }
	} elsif $mode == "monitor" {
	    maverick_network::interface_monitor { $name:
	        macaddress  => $macaddress,
	    }
	} elsif $mode == "ap" {
        # First create the underlying interface that hostapd will run on
	    maverick_network::interface_managed { $name:
	        type        => $type,
	        addressing  => "master",
	        macaddress  => $macaddress,
	        ipaddress   => $apaddress,
	        gateway     => $gateway,
	        nameservers => $nameservers,
	    }
	    # Then setup the AP
	    maverick_network::interface_ap { $name:
	        macaddress  => $macaddress,
	        ssid        => $ssid,
	        psk         => $psk,
            driver      => $driver,
            channel     => $channel,
            hw_mode     => $hw_mode,
            disable_broadcast_ssid => $disable_broadcast_ssid,
            dhcp_range  => $dhcp_range,
            dhcp_leasetime => $dhcp_leasetime,
	    }
	}
	# If not defined as monitor mode, ensure monitor disabled for this interface
	if $mode != "monitor" {
	    service_wrapper { "monitor-interface@${name}":
            ensure      => stopped,
            enable      => false,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/monitor-interface@.service"] ]
        }
	}
}
