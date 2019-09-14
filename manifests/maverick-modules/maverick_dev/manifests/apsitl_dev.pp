class maverick_dev::apsitl_dev (
    $instance_name = "dev",
    $instance_number = 0,
    $sitl_active = true,
    $sitl_port = 5000,
    $vehicle_type = "copter",
    $mavlink_proxy = "mavlink-router",
    $mavlink_active = true,
    $mavlink_startingtcp = 6000,
    $mavlink_tcpports = 3,
    $mavlink_startingudp = 14000,
    $mavlink_udpports = 3,
    $mavlink_udpinports = 3,
    $mavlink_serialout = undef,
    $mavlink_outbaud = 115200,
    $mavlink_outflow = false,
    $mavlink_replaceconfig = true,
    $ros_instance = true,
    $rosmaster_active = true,
    $rosmaster_port = 11000,
    $mavros_active = true,
    $mavros_startup_delay = 10,
    $api_instance = true,
    $api_active = true,
    $api_devmode = false,
    $api_debug = false,
    $api_replaceconfig = true,
) {

    # Remove old sitl setup
    service { "maverick-sitl":
        ensure      => stopped,
        enable      => false,
    } ->
    file { "/etc/systemd/system/maverick-sitl.service":
        ensure => absent,
        notify      => Exec["maverick-systemctl-daemon-reload"],
    } ->
    file { "/srv/maverick/software/maverick/bin/sitl.sh":
        ensure => absent,
    } ->
    # Rename old sitl setup config
    exec { "migrate-sitl-vehicle.conf":
        command => "/bin/mv /srv/maverick/config/dev/sitl-vehicle.conf /srv/maverick/config/dev/apsitl_${instance_name}-vehicle.conf",
        creates => "/srv/maverick/config/dev/apsitl_${instance_name}-vehicle.conf",
        onlyif  => "/bin/ls /srv/maverick/config/dev/sitl-vehicle.conf",
    } ->
    exec { "migrate-sitl.conf":
        command => "/bin/mv /srv/maverick/config/dev/sitl.conf /srv/maverick/config/dev/apsitl_${instance_name}.conf",
        creates => "/srv/maverick/config/dev/apsitl_${instance_name}.conf",
        onlyif  => "/bin/ls /srv/maverick/config/dev/sitl.conf",
    } ->
    exec { "migrate-sitl.screen.conf":
        command => "/bin/mv /srv/maverick/config/dev/sitl.screen.conf /srv/maverick/config/dev/apsitl_${instance_name}.screen.conf",
        creates => "/srv/maverick/config/dev/apsitl_${instance_name}.screen.conf",
        onlyif  => "/bin/ls /srv/maverick/config/dev/sitl.screen.conf",
    }

    maverick_dev::apsitl { $instance_name:
        instance_name       => $instance_name,
        instance_number     => $instance_number,
        sitl_active         => $sitl_active,
        rosmaster_active    => $rosmaster_active,
        mavlink_proxy       => $mavlink_proxy,
        mavros_active       => $mavros_active,
        mavlink_active      => $mavlink_active,
        mavlink_replaceconfig => $mavlink_replaceconfig,
        api_active          => $api_active,
        status_priority     => "154",
        api_devmode         => $api_devmode,
        api_debug           => $api_debug,
        api_replaceconfig   => $api_replaceconfig,
    }

}
