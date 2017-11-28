define maverick_network::interface_managed (
    $type = "ethernet",
    $addressing = "dhcp",
    $ipaddress = undef,
    $macaddress = undef,
    $gateway = undef,
    $netmask = undef,
    $nameservers = undef,
    $ssid = undef,
    $psk = undef,
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
                post_up         => ["/srv/maverick/software/maverick/bin/network-if-managed ${name}", "/sbin/dhclient $name"],
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
            post_up         => ["/srv/maverick/software/maverick/bin/network-if-managed ${name}"],
            options         => {
                wireless_mode       => "master",
            }
        }
    }
    
}