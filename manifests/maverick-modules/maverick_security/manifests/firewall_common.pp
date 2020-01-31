# @summary
#   Maverick_security::firewall_common class
#   This class declares standard common firewall rules
#
# @example Declaring the class
#   This class is included from maverick_security::firewall class and should not be included from elsewhere
#
class maverick_security::firewall_common {
    
    ### Allow ssh from everywhere
    firewall { '10 allow ssh access':
        dport        => 22,
        proto       => tcp,
        action      => accept,
    }
    
    ### Allow git traffic
    maverick_security::firewall::firerule { "git":
        ports       => [9148],
        ips         => lookup("firewall_ips"),
        proto       => "tcp", # allow both tcp and udp for rtsp and rtp
    }

}
