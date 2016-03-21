class maverick-network::wireless (
    ) {
    
    # Ensure wpasupplicant is installed
    package { "wpasupplicant":
        ensure      => installed
    }
    
    # Turn off predictable interface naming
    file { "/dev/null":
        ensure      => link,
        target      => "/etc/udev/rules.d/80-net-setup-link.rules",
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