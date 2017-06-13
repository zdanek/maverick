define maverick_ros::mavros (
    $rosmaster_port = "11311",
    $mavlink_port = "5770",
    $active = true,
    $mavros_startup_delay = 10,
) {
    
    file { "/srv/maverick/data/config/ros/mavros-${name}.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        mode        => "644",
        content     => template("maverick_ros/mavros.conf.erb"),
        notify      => Service_wrapper["maverick-mavros@${name}"],
    }
    
    if $active == true {
    	service_wrapper { "maverick-mavros@${name}":
            ensure      => running,
            enable      => true,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-mavros@.service"] ]
        }
    } else {
    	service_wrapper { "maverick-mavros@${name}":
            ensure      => stopped,
            enable      => false,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-mavros@.service"] ]
        }
    }
    
}