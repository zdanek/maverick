class maverick-network (
    $dnsclient = "disabled", 
    $ntpclient = "enabled",
    $wireless = true,
    ) {
    
    class { "network": 
        hostname => "${hostname}"
    }
    network::interface { 'lo':
        enable_dhcp     => false,
        auto            => true,
        method          => loopback,
        manage_order    => 0,
    }
    network::interface { 'eth0':
        enable_dhcp     => false,
        manage_order    => 10,
        auto            => false,
        method          => manual,
    }
    
    if $ntpclient == "enabled" {
        include maverick-network::ntpclient
    }
    
    if $dnsclient == "enabled" {
        include maverick-network::dnsclient
    }

    # Set high network buffers to better cope with our likely crappy usb ethernet and wifi links (12mb instead of default 128k)
    base::sysctl::conf { 
        "net.core.rmem_max": 							value => '12582912';
        "net.core.wmem_max": 							value => '12582912';
        "net.ipv4.tcp_rmem":							value => "10240 87380 12582912";
        "net.ipv4.tcp_wmem":							value => "10240 87380 12582912";
    }
    
    class { "maverick-network::wireless": }
    
}