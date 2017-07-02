define maverick_network::interface_monitor (
    $macaddress = undef,
) {

    # Define a basic interface with a post-up action to deal with the special monitor/injection setup
    network::interface { "$name":
        enable_dhcp     => false,
        auto            => true,
        allow_hotplug   => true,
        template        => "maverick_network/interface_fragment_wireless.erb",
        up              => ["/srv/maverick/software/maverick/bin/network-if-monitor"],
        options         => {
            # wireless_mode =>    "managed",
        },
    }

    # Add a config file for this interface
    file { "/srv/maverick/data/config/network/interface-${name}.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        replace     => false, # initialize but don't overwrite in the future
        source      => "puppet:///modules/maverick_network/interface.conf",
    }

}