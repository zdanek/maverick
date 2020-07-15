# @summary
#   Maverick_dev::apsitl class
#   This class declares the default Ardupilot SITL instance.  It then calls maverick_dev::apsitl that creates the actual SITL instance
#
# @example Declaring the class
#   This class is included from maverick_dev class and should not be included from elsewhere
#   It could be included selectively from eg. minimal environment.
#
# @param sitl_active
#   If true, the SITL service will be activated and enabled at boot time
# @param vehicle_type
#   Type of Ardupilot vehicle to use, eg. copter, plane, sub etc
# @param mavlink_proxy
#   Type of mavlink proxy to use, eg. mavlink-router, mavproxy or cmavnode
# @param mavlink_active
#   If true, the mavlink proxy will be activated and enabled at boot time
# @param mavlink_logging
#   If true and supported by the mavlink proxy software, then mavlink data will be logged to file
# @param mavlink_serialout
#   If set, proxy mavlink data out on this serial port
# @param mavlink_outbaud
#   Baud rate of mavlink_serialout port
# @param mavlink_outflow
#   If mavlink_serialout port should use hardware flow control
# @param ros_instance
#   If true, create a separate ROS instance for this SITL instance
# @param rosmaster_active
#   If true, set this separate ROS instance active and enabled at boot time
# @param mavros_active
#   If true, the separate MAVROS instance will be activated and enabled at boot time
# @param mavros_startup_delay
#   This delay causes Mavros to wait before starting, to give ROS and SITL time to boot fully first.  Should be increased on slower boards/environments.
# @param api_instance
#   If true, create a separate maverick-api instance
# @param api_active
#   If true, this maverick-api instance will be activated and enabled at boot time
# @param api_debug
#   If true, turn on the -api debug mode
# @param api_devmode
#   If true, turn on the -api dev mode
#
class maverick_dev::apsitl_apdev (
    $sitl_active = true,
    $vehicle_type = "copter",
    $mavlink_proxy = "mavlink-router",
    $mavlink_active = true,
    $mavlink_logging = true,
    $mavlink_serialout = undef,
    $mavlink_outbaud = 115200,
    $mavlink_outflow = false,
    $ros_instance = true,
    $rosmaster_active = true,
    $mavros_active = true,
    $mavros_startup_delay = 10,
    $api_instance = true,
    $api_active = false,
    $api_devmode = false,
    $api_debug = false,
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
    # Remove old sitl setup config
    file { ["/srv/maverick/config/dev/apsitl_dev-vehicle.conf", "/srv/maverick/config/dev/apsitl_dev.conf", "/srv/maverick/config/dev/apsitl_dev.screen.conf"]:
        ensure => absent,
    }

    # Remove old dev instance
    service { ["maverick-apsitl@dev", "maverick-mavlink@dev", "maverick-rosmaster@dev", "maverick-mavros@dev", "maverick-api@dev"]:
        ensure      => stopped,
        enable      => false,
    } ->
    file { "/srv/maverick/software/maverick/bin/status.d/154.dev":
        ensure      => absent,
        force       => true,
        recurse     => true,
    } ->
    maverick_dev::apsitl { "apdev":
        instance_name       => "apdev",
        instance_number     => 0,
        sitl_active         => $sitl_active,
        sitl_port           => 6110,
        mavlink_active      => $mavlink_active,
        mavlink_proxy       => $mavlink_proxy,
        mavlink_startingtcp => 6113,
        mavlink_tcpports    => 3,
        mavlink_startingudp => 6116,
        mavlink_udpports    => 3,
        mavlink_udpinports  => 3,
        mavlink_logging     => $mavlink_logging,
        mavlink_serialout   => $mavlink_serialout,
        mavlink_outbaud     => 115200,
        mavlink_outflow     => false,
        mavlink_replaceconfig => true,
        ros_instance        => $ros_instance,
        rosmaster_port      => 6111,
        rosmaster_active    => $rosmaster_active,
        mavros_active       => $mavros_active,
        mavros_startup_delay => $mavros_startup_delay,
        status_priority     => "154",
        status_entries      => true,
        api_instance        => $api_instance,
        api_name            => "APDev",
        api_port            => 6112,
        api_active          => $api_active,
        api_devmode         => $api_devmode,
        api_debug           => $api_debug,
        api_replaceconfig   => true,
    }

}
