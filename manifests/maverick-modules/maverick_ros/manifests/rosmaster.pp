define maverick_ros::rosmaster (
    $port = "11311",
    $active = true,
) {
    
    file { "/srv/maverick/data/config/ros/rosmaster-${name}.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        mode        => "644",
        content     => template("maverick_ros/rosmaster.conf.erb"),
        notify      => Service["maverick-rosmaster@${name}"],
    }

    if $active == true {
    	service { "maverick-rosmaster@${name}":
            ensure      => running,
            enable      => true,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-rosmaster@.service"] ]
        }
        # Punch some holes in the firewall for rosmaster
        if defined(Class["::maverick_security"]) {
            maverick_security::firewall::firerule { "rosmaster-${name}":
                ports       => ["${port}"],
                ips         => hiera("all_ips"),
                proto       => "tcp"
            }
        }
    } else {
    	service { "maverick-rosmaster@${name}":
            ensure      => stopped,
            enable      => false,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-rosmaster@.service"] ]
        }
    }

}