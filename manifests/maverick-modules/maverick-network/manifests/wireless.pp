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
    
    # Remove connman - ubuntu/intel connection manager.  Ugly and unwielding, we want a more controllable, consistent interface to networking.
    # Ensure This is done after the rest of wireless is setup otherwise we lose access to everything.
    if ($connman == false) {
        warning("Disabling connman connection manager: Please reset hardware and log back in if the connection hangs")
        file { "/etc/resolv.conf":
            ensure      => file,
            owner       => "root",
            group       => "root",
            mode        => 644,
        } ->
        service { "connmand":
            ensure      => stopped,
            enable      => false,
            require     => [Service["dhcpcd"], Network::Interface["eth0"], Network::Interface["wlan0"], Network::Interface["wlan1"]],
            notify      => Exec["connman-reboot"],
        } ->
        package { ["connman", "cmst"]:
            ensure      => absent
        } ->
        exec { "connman-reboot":
            refresh-only    => true,
            command         => "/sbin/reboot",
        }

    }    

}