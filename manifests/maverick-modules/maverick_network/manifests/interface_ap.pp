define maverick_network::interface_ap (
    $macaddress = undef,
    $ssid = "Maverick",
    $psk = "8097a204e44b0a740d5daad37d0e34ac16e4df353bc827dcd57d49b36d49740d",
    $driver = "nl80211",
    $channel = 1,
    $hw_mode = "g",
    $disable_broadcast_ssid = false,
    $dhcp_range = "192.168.10.10,192.168.10.50",
    $dhcp_leasetime = "24h",
    $forward = false,
) {
    ## Note, in order to keep this as simple as possible this AP setup does NOT allow for bridging, routing or NATing.
    ## It is a deliberately simple setup just to allow remote connection to the OBC for telemetry, video etc
    
    # Write a hostapd defaults file, this is necessary for the daemon to find the config
    file { "/etc/default/hostapd":
        ensure      => file,
        content     => template("maverick_network/hostapd.default.erb"),
        mode        => "644",
        owner       => "root",
        group       => "root",
        notify      => Service_wrapper["hostapd"]
    }
    
    # Write a hostadp.conf from template based on the parameters calculated and passed through from hiera
    file { "/etc/hostapd/hostapd.conf":
        ensure      => file,
        content     => template("maverick_network/hostapd.conf.erb"),
        mode        => "644",
        owner       => "root",
        group       => "root",
        notify      => Service_wrapper["hostapd"]
    }
    
    # Ensure hostapd service is running and enabled at boot.  This will probably blow up if
    #  more than one interface is configured to be an AP, but this is unlikely use case (hopefully)
    service_wrapper { "hostapd":
        ensure      => running,
        enable      => true,
        require     => [ File["/etc/hostapd/hostapd.conf"], File["/etc/default/hostapd"] ]
    }
    
    # If dnsmasq isn't already configured, configure it
    if $maverick_network::dnsmasq != true {
        class { "maverick_network::dnsmasq": }
    }

    ::dnsmasq::conf { "dnsmasq-dhcp":
        ensure      => present,
        prio        => 20,
        content     => template('maverick_network/dnsmasq-dhcp.erb'),
    }
    
    # Turn on IP forwarding and set iptables rules to forward data between the interfaces.
    if $forward != false {
        base::sysctl::conf {
            "net.ipv4.ip_forward":      value => 1;
            "net.ipv4.ip_dynaddr":      value => 1;
        }
        if defined(Class["::maverick_security"]) {
            firewall { "201 masquerade traffic from AP":
                chain       => 'POSTROUTING',
                jump        => 'MASQUERADE',
                proto       => 'all',
                outiface    => $forward,
                table       => 'nat',
            }
            firewall { '200 forward traffic through AP':
                chain       => 'FORWARD',
                action      => 'accept',
                proto       => 'all',
                iniface     => $name,
                outiface    => $forward,
            }
            maverick_security::firewall::firerule { "203 allow dns traffic from clients":
                ports       => [53],
                ips         => lookup("firewall_ips"),
                proto       => "udp",
            }
        }
    }

}
