define maverick_mavlink::mavlinkrouter (
    $input = "",
    $startingudp = 14560,
    $udpports = 5,
    $startingtcp = 5770,
    $tcpports = 5,
    $active = true,  
) {
 
    file { "/srv/maverick/data/config/mavlink/mavlinkrouter-${name}.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        replace     => false, # initialize but don't overwrite in the future
        content     => template("maverick_mavlink/mavlinkrouter.conf.erb"),
        notify      => Service["maverick-mavlinkrouter@${name}"],
    }
    file { "/srv/maverick/data/config/mavlink/mavlinkrouter-${name}.service.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        replace     => false, # initialize but don't overwrite in the future
        content     => template("maverick_mavlink/mavlinkrouter.service.conf.erb"),
        notify      => Service["maverick-mavlinkrouter@${name}"],
    }

    if $active == true {
    	service { "maverick-mavlinkrouter@${name}":
            ensure      => running,
            enable      => true,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-mavlinkrouter@.service"] ]
        }
        # Punch some holes in the firewall for mavlinkrouter
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
    	service { "maverick-mavlinkrouter@${name}":
            ensure      => stopped,
            enable      => false,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-mavlinkrouter@.service"] ]
        }
    }
    
}