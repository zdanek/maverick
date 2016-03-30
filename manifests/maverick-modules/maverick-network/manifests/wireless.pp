class maverick-network::wireless (
    $connman = false,
    ) {
    
    # Ensure wpasupplicant is installed
    package { "wpasupplicant":
        ensure      => installed
    } ->
    service { "wpa_supplicant":
        ensure      => stopped,
        enable      => false,
    } ->
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
    
    # Ensure rfkill installed and add a boot script to unblock all wlan interfaces
    package { "rfkill":
        ensure      => installed
    } ->
    file { "/etc/systemd/system/rfkill-unblock.service":
        ensure      => present,
        source      => "puppet:///modules/maverick-network/rfkill-unblock.service",
        mode        => 755,
        owner       => "root",
        group       => "root",
        notify      => Exec["maverick-systemctl-daemon-reload"],
    } ->
    service { "rfkill-unblock.service":
        enable      => true,
    }
    
    # Turn off predictable interface naming
    file { "/etc/udev/rules.d/80-net-setup-link.rules":
        ensure      => link,
        target      => "/dev/null",
    }
    
    
    # Define two wireless interfaces
    network::interface { "wlan0":
        enable_dhcp     => true,
        auto            => true,
        allow_hotplug   => true,
        method          => "dhcp",
        template        => "maverick-network/interface_fragment_wireless.erb",
        manage_order    => 20,
    }
    network::interface { "wlan1":
        enable_dhcp     => true,
        auto            => true,
        allow_hotplug   => true,
        method          => 'dhcp',
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