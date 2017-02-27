define maverick_mavlink::cmavnode (
    $input = "",
    $udpinaddress = "",
    $udpinport = "",
    $startingudp = 14560,
    $udpports = 5,
    $startingtcp = 5770,
    $tcpports = 5,
    $active = true,  
) {
 
    file { "/srv/maverick/data/config/cmavnode-${name}.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        replace     => false, # initialize but don't overwrite in the future
        content     => template("maverick_mavlink/cmavnode.conf.erb"),
        notify      => Service["maverick-cmavnode@${name}"],
    }
    file { "/srv/maverick/data/config/cmavnode-${name}.service.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        replace     => false, # initialize but don't overwrite in the future
        content     => template("maverick_mavlink/cmavnode.service.conf.erb"),
        notify      => Service["maverick-cmavnode@${name}"],
    }
    file { "/srv/maverick/data/config/cmavnode-${name}.screen.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        replace     => false,  # initialize but don't overwrite in the future
        content     => template("maverick_mavlink/cmavnode.screen.conf.erb"),
        notify      => Service["maverick-cmavnode@${name}"]
    }
    

    if $active == true {
    	service { "maverick-cmavnode@${name}":
            ensure      => running,
            enable      => true,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-cmavnode@.service"] ]
        }
        # Punch some holes in the firewall for cmavnode
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
    	service { "maverick-cmavnode@${name}":
            ensure      => stopped,
            enable      => false,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-cmavnode@.service"] ]
        }
    }
    
}