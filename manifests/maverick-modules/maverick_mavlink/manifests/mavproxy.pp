# @summary
#   This function creates an instance of a MAVProxy mavlink proxy.  It is called by other classes whenever a mavlink proxy is needed, such as flight controller or SITL connection.
#
# @example
#   @@maverick_mavlink::mavproxy { $instance_name:
#       ...
#   }
#
# @param active
#   If true, starts the maverick-mavlink@[instance] service and enables at boot.
# @param inputaddress
#   Serial or network address of incoming mavlink link.
# @param inputbaud
#   If using serial input, baud rate to use.
# @param inputflow
#   If using serial input, whether to use hardware flow control.
# @instance
#   This must be a unique number for each instance
# @param startingudp
#   Starting udp port number to listen on.
# @param udpports
#   Number of udp ports to define.
# @param udpinports
#   Number of udp in ports to define.
# @param startingtcp
#    Starting TCP port to listen on.
# @param tcpports
#   Number of TCP ports to listen on.
# @param serialout
#   If set, mavlink data will be proxied out to this serial address.
# @param outbaud
#   If set, defines the serial baud rate for serialout.
# @param outflow
#   If set, defines the serialout hardware flow control.
# @param replaceconfig
#   If true, fully manage the cmavnode instance configuration and overwrite each configure run.
#
define maverick_mavlink::mavproxy (
    Optional[Boolean] $active = undef,
    Optional[String] $inputaddress = "",
    Optional[Integer] $inputbaud = undef,
    Optional[Boolean] $inputflow = false,
    Integer $instance = 0,
    Integer $startingudp = 14570,
    Integer $udpports = 3,
    Integer $udpinports = 3,
    Integer $startingtcp = 5770,
    Integer $tcpports = 3,
    Optional[String] $serialout = undef,
    Optional[Integer] $outbaud = undef,
    Optional[Boolean] $outflow = false,
    Boolean $replaceconfig = true,
) {
    if $active == true {
        $service_notify = Service["maverick-mavlink@${name}"]
    } else {
        $service_notify = undef
    }
    if getvar("maverick_mavlink::mavcesium_install") == true {
        $mavcesium_installed = true
    } else {
        $mavcesium_installed = false
    }
    file { "/srv/maverick/config/mavlink/mavproxy-${name}.screen.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        replace     => $replaceconfig,  # initialize but don't overwrite in the future if false
        content     => template("maverick_mavlink/mavproxy.screen.conf.erb"),
        notify      => $service_notify,
    }

    if $active == true {
        file { "/srv/maverick/config/mavlink/mavlink-${name}.service.conf":
            content     => template("maverick_mavlink/mavproxy.service.conf.erb"),
            owner       => "mav",
            group       => "mav",
            notify      => Service["maverick-mavlink@${name}"],
        }
    	service { "maverick-mavlink@${name}":
            ensure      => running,
            enable      => true,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-mavlink@.service"] ]
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
    } elsif $active == false {
    	service { "maverick-mavlink@${name}":
            ensure      => stopped,
            enable      => false,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-mavlink@.service"] ]
        }
    }

}
