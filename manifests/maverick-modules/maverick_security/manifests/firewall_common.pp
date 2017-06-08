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
        ips         => hiera("firewall_ips"),
        proto       => "tcp", # allow both tcp and udp for rtsp and rtp
    }

}