# @summary
#   This function creates an instance of a monitor/injection interface.  It is called by process_interface for each monitor interface declared in the configuration.
#
# @example
#   @@maverick_network::interface_monitor { $instance_name:
#       ...
#   }
#
# @param macaddress
#   If set, locks onto the hardware/virtual interface with this MAC address.
#
define maverick_network::interface_monitor (
    Optional[String] $macaddress = undef,
) {

    # Define a basic interface with a post-up action to deal with the special monitor/injection setup
    network::interface { "$name":
        enable_dhcp     => false,
        auto            => true,
        allow_hotplug   => true,
        method          => "manual",
        template        => "maverick_network/interface_fragment_wireless.erb",
        post_up         => ["/srv/maverick/software/maverick/bin/network-if-config ${name} monitor"],
        options         => {
            wireless_mode =>    "monitor",
        },
    }

    # Add a config file for this interface
    file { "/srv/maverick/config/network/interface-${name}.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        replace     => false, # initialize but don't overwrite in the future
        source      => "puppet:///modules/maverick_network/interface.conf",
    }

}
