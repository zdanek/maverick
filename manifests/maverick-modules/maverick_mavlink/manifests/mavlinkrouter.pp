define maverick_mavlink::mavlinkrouter (
    $inputtype = "serial",
    $inputaddress = undef,
    $inputport = undef,
    $startingudp = 14560,
    $udpports = 5,
    $startingtcp = 5770,
    $tcpports = 5,
    $active = true,
    $replaceconfig = true,
) {
    if $active {
        $notify = Service["maverick-mavlinkrouter@${name}"]
    } else {
        $notify = undef
    }
    file { "/srv/maverick/data/config/mavlink/mavlinkrouter-${name}.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        replace     => $replaceconfig, # initialize but don't overwrite in the future if false
        content     => template("maverick_mavlink/mavlinkrouter.conf.erb"),
        notify      => $notify,
    }
    file { "/srv/maverick/data/config/mavlink/mavlinkrouter-${name}.service.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        replace     => $replaceconfig, # initialize but don't overwrite in the future if false
        content     => template("maverick_mavlink/mavlinkrouter.service.conf.erb"),
        notify      => $notify,
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