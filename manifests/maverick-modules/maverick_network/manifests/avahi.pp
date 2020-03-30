# @summary
#   Maverick_network::Avahi class
#   This class installs/manages Avahi zeroconf software/service.
#
# @example Declaring the class
#   This class is included from maverick_network class and should not be included from elsewhere
#
# @param explicit_naming
#   If false, turns on 'publish-addresses' avahi config setting. 
#
class maverick_network::avahi (
    Boolean $explicit_naming = false,
) {

    # Lookup/set hostname and ipaddress    
    if ! empty(lookup("primaryip")) {
        $_ipaddress = lookup("primaryip")
    } else {
        $_ipaddress = $::ipaddress
    }
    if ! empty(lookup("hostname")) {
        $_hostname = lookup("hostname")
    } else {
        $_hostname = $::hostname
    }

    ensure_packages(["avahi-daemon", "avahi-utils"])
    package { ["avahi-autoipd", "avahi-dnsconfd"]:
        ensure      => purged,
    } ->
    
    file { "/etc/avahi/avahi-daemon.conf":
        owner       => root,
        group       => root,
        mode        => "644",
        content     => template("maverick_network/avahi-daemon.conf.erb"),
        notify      => Service["avahi-daemon"],
        require     => Package["avahi-daemon"],
    } ->
    service { "avahi-daemon":
        ensure      => running,
        enable      => true
    }
    
    # Declare /etc/avahi/hosts and then build up through interface processing, one per interface
    concat { "/etc/avahi/hosts":
        ensure          => present,
        ensure_newline  => true,
        notify          => Service["avahi-daemon"],
    }
    
    concat::fragment { "avahi-hosts-main":
        target      => "/etc/avahi/hosts",
        content     => "# Avahi hosts is controlled by Maverick/Puppet, any modifications will be overridden on the next configure run",
    }

    # Add a static mapping for our hostname
    # Update: back this out, othewrise avahi doesn't respond to dynamic changes in IP 
    /*
    concat::fragment { "avahi-hosts-ourip":
        target      => "/etc/avahi/hosts",
        content     => "${_ipaddress}   ${hostname}.local",
    }
    */

    # Allow udp port 5353 for mdns requests
    if defined(Class["::maverick_security"]) {
        maverick_security::firewall::firerule { "avahi":
            ports       => [5353],
            ips         => [],
            proto       => "udp"
        }
    }   
}
