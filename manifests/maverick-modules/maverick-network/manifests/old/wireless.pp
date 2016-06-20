class maverick-network::wireless (
    $type = "infrastructure", # 'infrastructure' for normal wifi, 'ap' to act as an Access Point, 'broadcast' for wifibroadcast
    $wlan0_name = "wlan0",
    $wlan0_mode = "dhcp",
    $wlan0_address = undef,
    $wlan0_netmask = undef,
    $wlan0_gateway = undef,
    $wlan0_macaddress = undef,
    $wlan0_ssid = undef,
    $wlan0_psk = undef,
    $wlan1_name = "wlan1",
    $wlan1_mode = "dhcp",
    $wlan1_address = undef,
    $wlan1_netmask = undef,
    $wlan1_gateway = undef,
    $wlan1_macaddress = undef,
    $wlan1_ssid = undef,
    $wlan1_psk = undef,
) {

    # Ensure wpasupplicant is installed, but not running
    package { "wpasupplicant":
        ensure      => installed
    } ->
    service { "wpa_supplicant":
        ensure      => stopped,
        enable      => false,
    }
    
    # Ensure rfkill installed and add a boot script to unblock all wlan interfaces
    package { "rfkill":
        ensure      => installed
    } ->
    file { "/etc/systemd/system/rfkill-unblock.service":
        ensure      => present,
        source      => "puppet:///modules/maverick-network/rfkill-unblock.service",
        mode        => 644,
        owner       => "root",
        group       => "root",
        notify      => Exec["maverick-systemctl-daemon-reload"],
    } ->
    service { "rfkill-unblock.service":
        enable      => true,
    }

    # Define first wireless interface
    if $wlan0_ssid {
        $_wlan0_ssid = $wlan0_ssid
    } elsif hiera('wifi_ssid') {
        $_wlan0_ssid = hiera('wifi_ssid')
    } else {
        $_wlan0_ssid = undef
    }
    if $wlan0_psk {
        $_wlan0_psk = $wlan0_psk
    } elsif hiera('wifi_psk') {
        $_wlan0_psk = hiera('wifi_psk')
    } else {
        $_wlan0_psk = undef
    }
    if $wlan0_mode == "dhcp" {
        network::interface { "$wlan0_name":
            manage_order    => 20,
            enable_dhcp     => true,
            auto            => true,
            allow_hotplug   => true,
            method          => "dhcp",
            template        => "maverick-network/interface_fragment_wireless.erb",
            wpa_ssid        => $_wlan0_ssid,
            wpa_psk         => $_wlan0_psk,
        }
    } else {
        network::interface { "$wlan0_name":
            manage_order    => 20,
            enable_dhcp     => false,
            auto            => true,
            allow_hotplug   => true,
            method          => "static",
            template        => "maverick-network/interface_fragment_wireless.erb",
            wpa_ssid        => $_wlan0_ssid,
            wpa_psk         => $_wlan0_psk,
            ipaddress       => $wlan0_address,
            netmask         => $wlan0_netmask,
            gateway         => $wlan0_gateway,
            hwaddr          => $wlan0_macaddress,
        }
    }
    
    # Define second wireless interface
    if $wlan1_ssid {
        $_wlan1_ssid = $wlan1_ssid
    } elsif hiera('wifi_ssid') {
        $_wlan1_ssid = hiera('wifi_ssid')
    } else {
        $_wlan1_ssid = undef
    }
    if $wlan1_psk {
        $_wlan1_psk = $wlan0_psk
    } elsif hiera('wifi_psk') {
        $_wlan1_psk = hiera('wifi_psk')
    } else {
        $_wlan1_psk = undef
    }
    if $wlan1_mode == "dhcp" {
        network::interface { "$wlan1_name":
            manage_order    => 21,
            enable_dhcp     => true,
            auto            => true,
            allow_hotplug   => true,
            method          => "dhcp",
            template        => "maverick-network/interface_fragment_wireless.erb",
            wpa_ssid        => $_wlan1_ssid,
            wpa_psk         => $_wlan1_psk,
        }
    } else {
        network::interface { "$wlan1_name":
            manage_order    => 21,
            enable_dhcp     => false,
            auto            => true,
            allow_hotplug   => true,
            method          => "static",
            template        => "maverick-network/interface_fragment_wireless.erb",
            wpa_ssid        => $_wlan1_ssid,
            wpa_psk         => $_wlan1_psk,
            ipaddress       => $wlan1_address,
            netmask         => $wlan1_netmask,
            gateway         => $wlan1_gateway,
            hwaddr          => $wlan1_macaddress,
        }
    }

    # If wireless auth defaults are set in localconf.json, configure it 
    # Retrieve wireless auth data from hiera
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