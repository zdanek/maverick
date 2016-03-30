class maverick-network::ethernet (
    $eth0_name = "eth0",
    $eth0_mode = "dhcp",
    $eth0_address = undef,
    $eth0_netmask = undef,
    $eth0_gateway = undef,
    $eth0_macaddress = undef,
) {

    if $eth0_mode == "dhcp" {
        network::interface { "$eth0_name":
            enable_dhcp     => true,
            manage_order    => 10,
            auto            => true,
            method          => "dhcp",
        }
    } else {
        network::interface { "$eth0_name":
            enable_dhcp     => false,
            manage_order    => 10,
            auto            => true,
            method          => "static",
            ipaddress       => $eth0_address,
            netmask         => $eth0_netmask,
            gateway         => $eth0_gateway,
            hwaddr          => $eth0_macaddress,
        }
    }
    
}