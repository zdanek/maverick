class maverick-network (
    $dnsclient = "disabled", 
    $ntpclient = "enabled",
    ) {
    
    class { "network": 
        hostname => "${hostname}"
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
}