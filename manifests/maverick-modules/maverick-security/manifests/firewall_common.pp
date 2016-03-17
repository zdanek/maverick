class maverick-security::firewall_common {
    
    ### Allow ssh from everywhere
    firewall { '10 allow ssh access':
        dport        => 22,
        proto       => tcp,
        action      => accept,
    }

}