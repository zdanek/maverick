define maverick_mavlink::cmavnode (
    $inputaddress = undef,
    $inputbaud = 115200,
    $udpinaddress = undef,
    $udpinport = undef,
    $startingudp = 14570,
    $udpports = 3,
    $udpinports = 3,
    $startingtcp = 5770,
    $tcpports = 3,
    $active = true,
    $replaceconfig = true,
) {
    if $active {
        $service_notify = Service_wrapper["maverick-cmavnode@${name}"]
    } else {
        $service_notify = undef
    }
    file { "/srv/maverick/config/mavlink/cmavnode-${name}.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        replace     => $replaceconfig, # initialize but don't overwrite in the future if false
        content     => template("maverick_mavlink/cmavnode.conf.erb"),
        notify      => $service_notify,
    }
    file { "/srv/maverick/config/mavlink/cmavnode-${name}.service.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        replace     => $replaceconfig, # initialize but don't overwrite in the future if false
        content     => template("maverick_mavlink/cmavnode.service.conf.erb"),
        notify      => $service_notify,
    }
    file { "/srv/maverick/config/mavlink/cmavnode-${name}.screen.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        replace     => $replaceconfig,  # initialize but don't overwrite in the future if false
        content     => template("maverick_mavlink/cmavnode.screen.conf.erb"),
        notify      => $service_notify,
    }

    if $active == true {
    	service_wrapper { "maverick-cmavnode@${name}":
            ensure      => running,
            enable      => true,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-cmavnode@.service"] ]
        }
        # Punch some holes in the firewall for cmavnode
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
    	service_wrapper { "maverick-cmavnode@${name}":
            ensure      => stopped,
            enable      => false,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-cmavnode@.service"] ]
        }
    }
    
}