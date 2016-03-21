class maverick-network::wireless (
    ) {
    
    # Ensure wpasupplicant is installed
    package { "wpasupplicant":
        ensure      => installed
    } ->
    service { "wpa_supplicant":
        ensure      => stopped,
        enable      => false,
    }
    
    service { "udhcpd":
        ensure      => stopped,
        enable      => false
    } ->
    package { "dhcpcd5":
        ensure      => installed
    }->
    service { "dhcpcd":
        ensure      => running,
        enable      => true,
        require     => Class["Network"]
    }
    
    # Turn off predictable interface naming
    file { "/etc/udev/rules.d/80-net-setup-link.rules":
        ensure      => link,
        target      => "/dev/null",
    }
    
    # Define two wireless interfaces
    network::interface { "wlan0":
        enable_dhcp     => false,
        auto            => false,
        allow_hotplug   => true,
        method          => "manual",
        template        => "maverick-network/interface_fragment_wireless.erb",
        manage_order    => 20,
    }
    network::interface { "wlan1":
        enable_dhcp     => false,
        auto            => false,
        allow_hotplug   => true,
        method          => 'manual',
        template        => "maverick-network/interface_fragment_wireless.erb",
        manage_order    => 21,
    }

    # If a wireless NIC is detected and defaults are set in localconf.json, configure it 
    $wifi_ssid = hiera('wifi_ssid')
    $wifi_passphrase = hiera('wifi_passphrase')
    if $wifi_ssid and $wifi_passphrase {
        file { "/etc/wpa_supplicant/wpa_supplicant.conf":
            content => template("maverick-network/wpa_supplicant.conf.erb"),
            mode    => 600,
            owner   => "root",
            group   => "root",
        }
    }
    
}