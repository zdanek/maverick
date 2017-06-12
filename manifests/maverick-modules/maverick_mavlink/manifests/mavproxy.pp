define maverick_mavlink::mavproxy (
    $inputaddress = "",
    $inputbaud = undef,
    $instance = 0,
    $startingudp = 14570,
    $udpports = 3,
    $udpinports = 3,
    $startingtcp = 5770,
    $tcpports = 3,
    $active = true,
    $replaceconfig = true,
) {
    if $active {
        $service_notify = Service["maverick-mavproxy@${name}"]
    } else {
        $service_notify = undef
    }
    file { "/srv/maverick/data/config/mavlink/mavproxy-${name}.service.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        replace     => $replaceconfig, # initialize but don't overwrite in the future if false
        content     => template("maverick_mavlink/mavproxy.service.conf.erb"),
        notify      => $service_notify,
    }
    file { "/srv/maverick/data/config/mavlink/mavproxy-${name}.screen.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        replace     => $replaceconfig,  # initialize but don't overwrite in the future if false
        content     => template("maverick_mavlink/mavproxy.screen.conf.erb"),
        notify      => $service_notify,
    }

    if $active == true {
    	service { "maverick-mavproxy@${name}":
            ensure      => running,
            enable      => true,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-mavproxy@.service"] ]
        }
        # Punch some holes in the firewall for mavproxy
        if defined(Class["::maverick_security"]) {
            $endingudp = $startingudp + $udpports + $udpinports
            maverick_security::firewall::firerule { "mavlink-${name}-udp":
                ports       => ["${startingudp}-${endingudp}"],
                ips         => lookup("firewall_ips"),
                proto       => "udp"
            }
            $endingtcp = $startingtcp + $tcpports
            maverick_security::firewall::firerule { "mavlink-${name}-tcp":
                ports       => ["${startingtcp}-${endingtcp}"],
                ips         => lookup("firewall_ips"),
                proto       => "tcp"
            }
        }
    } else {
    	service { "maverick-mavproxy@${name}":
            ensure      => stopped,
            enable      => false,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-mavproxy@.service"] ]
        }
    }

}