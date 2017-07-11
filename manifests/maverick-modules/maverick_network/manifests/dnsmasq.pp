class maverick_network::dnsmasq (
) {
   
    # This depends on a forked puppet module: https://github.com/mrjoshuap/puppet-dnsmasq
    
    # dnsmasq::purge is set to false by default in maverick/conf/puppet-defaults.json,
    # because oddly specifically we need README in /etc/dnsmasq.d.  It should remain from 
    # package installation, so ensure we don't purge it and replace it just in case.
    class { "::dnsmasq": } ->
    file { "/etc/dnsmasq.d/README":
        ensure      => present,
        content     => template("maverick_network/dnsmasq-README")
    } ->
    # Add a fix in the for systemd dependency, see https://github.com/fnoop/maverick/issues/521
    file { "/lib/systemd/system/dnsmasq.service":
        ensure      => present,
        source      => "puppet:///modules/maverick_network/dnsmasq.service.fixed",
    }
    
    ::dnsmasq::conf { "dnsmasq-basic":
        ensure      => present,
        content     => template('maverick_network/dnsmasq-core.erb'),
    }
    
    # Allow udp port 67 for dhcp requests
    if defined(Class["::maverick_security"]) {
        maverick_security::firewall::firerule { "dhcpd":
            ports       => [67],
            ips         => [],
            proto       => "udp"
        }
    }

}