define maverick_mavlink::mavproxy (
    $input = "",
    $instance = 0,
    $startingudp = 14560,
    $udpports = 5,
    $startingtcp = 5770,
    $tcpports = 5,
    $active = true,
) {
    file { "/srv/maverick/data/config/mavproxy-${name}.service.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        replace     => false, # initialize but don't overwrite in the future
        content     => template("maverick_mavlink/mavproxy.service.conf.erb"),
        notify      => Service["maverick-mavproxy@${name}"],
    }
    file { "/srv/maverick/data/config/mavproxy-${name}.screen.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        replace     => false,  # initialize but don't overwrite in the future
        content     => template("maverick_mavlink/mavproxy.screen.conf.erb"),
        notify      => Service["maverick-mavproxy@${name}"]
    }

    if $active == true {
    	service { "maverick-mavproxy@${name}":
            ensure      => running,
            enable      => true,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-mavproxy@.service"] ]
        }
        # Punch some holes in the firewall for mavproxy
        if defined(Class["::maverick_security"]) {
            $endingudp = $startingudp + $udpports
            maverick_security::firewall::firerule { "mavlink-${name}-udp":
                ports       => ["${startingudp}-${endingudp}"],
                ips         => hiera("all_ips"),
                proto       => "udp"
            }
            $endingtcp = $startingtcp + $tcpports
            maverick_security::firewall::firerule { "mavlink-${name}-tcp":
                ports       => ["${startingtcp}-${endingtcp}"],
                ips         => hiera("all_ips"),
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