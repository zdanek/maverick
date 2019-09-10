define maverick_mavlink::mavlink_router (
    $inputtype = "serial",
    $inputaddress = undef,
    $inputbaud = undef,
    $inputflow = false,
    $inputport = undef,
    $startingudp = 14570,
    $udpports = 3,
    $udpinports = 3,
    $startingtcp = 5770,
    $tcpports = 3,
    $serialout = undef,
    $outbaud = undef,
    $outflow = false,
    $active = undef,
    $logging = true,
    $replaceconfig = true,
) {
    if $active == true {
        $service_notify = Service["maverick-mavlink@${name}"]
    } else {
        $service_notify = undef
    }
    file { "/srv/maverick/config/mavlink/mavlink-router-${name}.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        replace     => $replaceconfig, # initialize but don't overwrite in the future if false
        content     => template("maverick_mavlink/mavlink-router.conf.erb"),
        notify      => $service_notify,
    }

    if $active == true {
        file { "/srv/maverick/config/mavlink/mavlink-${name}.service.conf":
            content     => template("maverick_mavlink/mavlink-router.service.conf.erb"),
            owner       => "mav",
            group       => "mav",
            notify      => Service["maverick-mavlink@${name}"],
        }
    	service { "maverick-mavlink@${name}":
            ensure      => running,
            enable      => true,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-mavlink@.service"] ]
        }
        # Punch some holes in the firewall for mavlink-router
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
    } elsif $active == false {
    	service { "maverick-mavlink@${name}":
            ensure      => stopped,
            enable      => false,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-mavlink@.service"] ]
        }
    }
    
}
