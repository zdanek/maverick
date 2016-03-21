class maverick-network::wireless (
    ) {
    
    # Ensure wpasupplicant is installed
    package { "wpasupplicant":
        ensure      => installed
    } ->
    #service { "wpa_supplicant":
    #    ensure      => running,
    #    enable      => true
    #}
    
    service { "udhcpd":
        ensure      => stopped,
        enable      => false
    } ->
    package { "dhcpcd5":
        ensure      => installed
    }->
    service { "dhcpcd":
        ensure      => running,
        enable      => true
    }
    
    # Turn off predictable interface naming
    file { "/etc/udev/rules.d/80-net-setup-link.rules":
        ensure      => link,
        target      => "/dev/null",
    }
    
    # If a wireless NIC is detected and defaults are set in localconf.json, configure it 
    $wifi_ssid = hiera('wifi_ssid')
    $wifi_passphrase = hiera('wifi_passphrase')
    if $interfaces =~ /wlan/ and $wifi_ssid and $wifi_passphrase {
        file { "/etc/wpa_supplicant/wpa_supplicant.conf":
            content => template("maverick-network/wpa_supplicant.conf.erb"),
            mode    => 600,
            owner   => "root",
            group   => "root",
        }
    }
    
}