define maverick_network::interface_monitor (
    $macaddress = undef,
) {

    # Define a service definition for this interface
	service { "monitor-interface@${name}":
        ensure      => running,
        enable      => true,
        require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/monitor-interface@.service"] ]
    }

    # Add a config file for this interface
    file { "/srv/maverick/data/config/network/monitor-interface-${name}.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        replace     => false, # initialize but don't overwrite in the future
        source      => "puppet:///modules/maverick_network/monitor-interface.conf",
    }

}