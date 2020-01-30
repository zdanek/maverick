# @summary
#   This function creates an instance of a managed ethernet or wireless interface.  It is called by process_interface for each managed interface declared in the configuration.
#
# @example
#   @@maverick_network::interface_managed { $instance_name:
#       ...
#   }
#
# @param type
#	Defines the interface type - ethernet or wireless.
# @param addressing
#	Defines the type of addressing for the internet - dhcp, static or master.
# @param ipaddress
#   If set, defines a static IP address for this interface.
# @param macaddress
#   If set, locks onto the hardware/virtual interface with this MAC address.
# @param gateway
#		If set, defines a gateway to be used for this interface.
# @param netmask
#		If set, defines the netmask for this interface.
#	@param nameservers
#		If set, defines a set of namservers to be used for this interface.
# @param ssid
#   The SSID to use to create this AP.
# @psk
#   The PSK to use to create this AP, that associating clients will need to know in order to associate.
#
define maverick_network::interface_managed (
    Enum['ethernet', 'wireless'] $type = "ethernet",
    Enum['dhcp', 'static', 'master'] $addressing = "dhcp",
    Optional[String] $ipaddress = undef,
    Optional[String] $macaddress = undef,
    Optional[String] $gateway = undef,
    Optional[String] $netmask = undef,
    Optional[String] $nameservers = undef,
    Optional[String] $ssid = undef,
    Optional[String] $psk = undef,
) {
	
	# If IP address is specified, then add it to avahi hosts
	if $ipaddress {
	    concat::fragment { "avahi-hosts-${name}":
            target      => "/etc/avahi/hosts",
            content     => "${ipaddress} ${hostname}-${name}.local",
        }
	}
	
	if $type == "wireless" {
	    # Add a config file for this interface
        file { "/srv/maverick/config/network/interface-${name}.conf":
            ensure      => present,
            owner       => "mav",
            group       => "mav",
            replace     => false, # initialize but don't overwrite in the future
            source      => "puppet:///modules/maverick_network/interface.conf",
        }
    }
    
	# Define first wireless interface
    if $ssid {
        $_ssid = $ssid
    } elsif !empty(lookup('wifi_ssid')) {
        $_ssid = lookup('wifi_ssid')
    } else {
        $_ssid = undef
    }
    if $psk {
        $_psk = $psk
    } elsif !empty(lookup('wifi_psk')) {
        $_psk = lookup('wifi_psk')
    } else {
        $_psk = undef
    }
    
    if $addressing == "dhcp" {
        if $type == "wireless" {
            network::interface { "$name":
                enable_dhcp     => true,
                #manage_order    => 10,
                auto            => true,
                allow_hotplug   => true,
                method          => "dhcp",
                template        => "maverick_network/interface_fragment_wireless.erb",
                wpa_ssid        => $_ssid,
                wpa_psk         => $_psk,
                post_up         => ["/srv/maverick/software/maverick/bin/network-if-config ${name} managed"],
                options         => {},
            }
        } elsif $type == "ethernet" {
            network::interface { "$name":
                enable_dhcp     => true,
                #manage_order    => 10,
                auto            => true,
                allow_hotplug   => true,
                method          => "dhcp",
                #options         => {},
            }
        }
    } elsif $addressing == "static" {
        if $type == "wireless" {
            network::interface { "$name":
                enable_dhcp     => false,
                #manage_order    => 10,
                auto            => true,
                allow_hotplug   => true,
                method          => "static",
                ipaddress       => $ipaddress,
                netmask         => $netmask,
                gateway         => $gateway,
                dns_nameservers => $nameservers,
                template        => "maverick_network/interface_fragment_wireless.erb",
                wpa_ssid        => $_ssid,
                wpa_psk         => $_psk,
                options         => {},
            }
        } elsif $type == "ethernet" {
            network::interface { "$name":
                enable_dhcp     => false,
                #manage_order    => 10,
                auto            => true,
                allow_hotplug   => true,
                method          => "static",
                ipaddress       => $ipaddress,
                netmask         => $netmask,
                gateway         => $gateway,
                dns_nameservers => $nameservers,
                #options         => {}
            }
        }
    } elsif $addressing == "master" {
        network::interface { "$name":
            enable_dhcp     => false,
            auto            => true,
            allow_hotplug   => true,
            method          => "static",
            ipaddress       => $ipaddress,
            netmask         => $netmask,
            template        => "maverick_network/interface_fragment_wireless.erb",
            post_up         => ["/srv/maverick/software/maverick/bin/network-if-config ${name} managed"],
            options         => {
                wireless_mode       => "master",
            }
        }
    }
    
}
