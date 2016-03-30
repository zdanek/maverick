class maverick-network (
    $dnsclient = "disabled", 
    $ntpclient = "enabled",
    $ethernet = true,
    $wireless = true,
    $netman = false,
    $predictable = false,
    ) {

    # Make sure dhcp servers are turned off
    service { "udhcpd":
        ensure      => stopped,
        enable      => false
    } ->
    package { "dhcpcd5":
        ensure      => installed
    } ->
    service { "dhcpcd":
        ensure      => stopped,
        enable      => false,
        require     => Class["Network"]
    }
    
    # Base network setup
    class { "network": 
        hostname => "${hostname}",
        config_file_notify => '',
    }
    network::interface { 'lo':
        enable_dhcp     => false,
        auto            => true,
        method          => loopback,
        manage_order    => 0,
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
    
    # Turn off predictable interface naming
    if $predictable == false {
        file { "/etc/udev/rules.d/80-net-setup-link.rules":
            ensure      => link,
            target      => "/dev/null",
        }
    }
    
    if $ethernet {
        class { "maverick-network::ethernet": }
    }
    
    if $wireless {
        class { "maverick-network::wireless": }
    }
    
    # Remove connman - ubuntu/intel connection manager.  Ugly and unwielding, we want a more controllable, consistent interface to networking.
    # Ensure This is done after the rest of wireless is setup otherwise we lose access to everything.
    if $netman == false and $netman_connmand == "yes" {
        warning("Disabling connman connection manager: Please reset hardware and log back in if the connection hangs.  Please reboot when maverick is completed to activate new network config.")
        file { "/etc/resolv.conf":
            ensure      => file,
            owner       => "root",
            group       => "root",
            mode        => 644,
        } ->
        service { "connman.service":
            ensure      => undef,
            enable      => false,
        }
    } 
    
    # Remove NetworkManager
    if $netman == false and $netman_networkmanager == "yes" {
        warning("Disabling NetworkManager connection manager: Please reset hardware and log back in if the connection hangs.  Please reboot when maverick is completed to activate new network config.")
        service { "NetworkManager":
            ensure      => stopped,
            enable      => false,
        } ->
        package { "network-manager":
            ensure      => absent, # remove but don't purge, so it can be restored later
        }
    }
    
}