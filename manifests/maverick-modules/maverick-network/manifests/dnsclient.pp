class maverick-network::dnsclient (
        #$servers = ['8.8.8.8', '8.8.4.4', '8.26.56.26', '8.20.247.20', '209.244.0.3', '209.244.0.4'],
        $servers = ['192.168.1.254'],
        $domain = "home",
    ) {

    class { "::dnsclient":
        nameservers     => $servers,
        domain          => $domain,
    }
    
    package { "bind-utils" :
        ensure => installed,
    }

}