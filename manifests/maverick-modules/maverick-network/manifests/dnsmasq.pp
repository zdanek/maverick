class maverick-network::dnsmasq (
) {
   
    include ::dnsmasq
    # dnsmasq::purge is set to false by default, because we bizarrely need README in /etc/dnsmasq.d
    file { "/etc/dnsmasq.d/README":
        ensure      => present,
        content     => template("maverick-network/dnsmasq-README")
    }
    
    # Allow udp port 67 for dhcp requests
    if defined(Class["::maverick-security"]) {
        maverick-security::firewall::firerule { "dhcpd":
            ports       => [67],
            ips         => [],
            proto       => "udp"
        }
    }

}