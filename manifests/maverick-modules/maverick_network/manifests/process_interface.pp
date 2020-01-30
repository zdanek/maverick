# @summary
#   This function creates an instance of a network interface.  It is called by maverick_network for each interface declared in the configuration.
#
# @example
#   @@maverick_network::process_interface { $instance_name:
#       ...
#   }
#
# @param mode
#   Defines the interface mode.  Currently managed, monitor and ap are supported.
# @param type
#		Defines the interface type - ethernet or wireless.
# @param addressing
#		Defines the type of addressing for the internet - dhcp, static or master.
# @param macaddress
#		If set, locks the MAC address to use for this interface.
#	@param ipaddress
# 	If set, defines a static IP address for this interface.
# @param apaddress
#		If defining an AP interface, this sets the master address of the interface.
# @param gateway
#		If set, defines a gateway to be used for this interface.
# @param netmask
#		If set, defines the netmask for this interface.
#	@param nameservers
#		If set, defines a set of namservers to be used for this interface.
#	@param ssid
#		If defining a wireless managed interface, set the SSID for association.  If defining an AP interface, set the SSID to be served.
# @param psk
#		If defining a wireless managed interface, set the PSK for association.  If defining an AP interface, set the PSK to be used for client association.
# @param driver
#		Set the kernel driver to be used.
# @param channel
#		If set, define a fixed wireless channel to be used.
# @param hw_mode
#		Define the wireless mode.
# @param disable_broadcast_ssid
#		If true, do not broadcast the SSID if an AP.
# @param dhcp_range
#		If defining a wireless AP, use this range of IP addresses for associated clients
# @param dhcp_leasetime
#		Length of lease time to use for dhcp server
# @param forward
#		If defining an AP interface, turn on network forwarding to bridge network flow between interfaces.
#
define maverick_network::process_interface (
    Enum['managed', 'monitor', 'ap'] $mode = "managed",
    Enum['ethernet', 'wireless'] $type = "ethernet",
    Enum['dhcp', 'static', 'master'] $addressing = "dhcp",
    Optional[String] $macaddress = undef,
    Optional[String] $ipaddress = undef,
    Optional[String] $apaddress = "192.168.10.1",
    Optional[String] $gateway = undef,
    Optional[String] $netmask = undef,
    Optional[Array[String]] $nameservers = undef,
    Optional[String] $ssid = undef,
    Optional[String] $psk = undef,
    String $driver  = "nl80211",
    Optional[String] $channel = undef,
    String $hw_mode = "g",
    Boolean $disable_broadcast_ssid = false,
    String $dhcp_range = "192.168.10.10,192.168.10.50",
    String $dhcp_leasetime = "24h",
    Boolean $forward = false,
) {
	# Display a warning to reboot if networking config has chagned significantly
    if (!(empty($macaddress)) and $macaddress !~ "xx") and ((getvar("::netinfo_${name}_macaddress") != $macaddress) or (!empty($::netinfo_interfaces) and !($name in $::netinfo_interfaces) and $name != "eth0" and $name != "wlan0")) {
    	warning("WARNING: Interface config has changed significantly, PLEASE REBOOT TO ACTIVATE")
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
	        netmask		=> $netmask,
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
	        netmask		=> $netmask,
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
            forward		=> $forward,
	    }
	}
	# If not defined as monitor mode, ensure monitor disabled for this interface
	if $mode != "monitor" {
	    service { "monitor-interface@${name}":
            ensure      => stopped,
            enable      => false,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/monitor-interface@.service"] ]
        }
	}
}
