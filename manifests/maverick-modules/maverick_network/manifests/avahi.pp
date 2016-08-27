class maverick_network::avahi (
    $explicit_naming = false,
) {
    
    ensure_packages(["avahi-daemon", "avahi-utils"])
    
    file { "/etc/avahi/avahi-daemon.conf":
        owner       => root,
        group       => root,
        mode        => 644,
        content     => template("maverick_network/avahi-daemon.conf.erb"),
        notify      => Service["avahi-daemon"]
    } ->
    service { "avahi-daemon":
        ensure      => running,
        enable      => true
    }
    
    # Declare /etc/avahi/hosts and then build up through interface processing, one per interface
    concat { "/etc/avahi/hosts":
        ensure      => present,
    }

    # Allow udp port 5353 for mdns requests
    if defined(Class["::maverick_security"]) {
        maverick_security::firewall::firerule { "avahi":
            ports       => [5353],
            ips         => [],
            proto       => "udp"
        }
    }   
}