# @summary
#   Rosmaster is the core of the ROS 1 system.  It is responsible for managing names and registration of nodes on the system.
#   This function creates a rosmaster instance.  It is typically called by modules that also create mavlink proxy and mavros instances.
#
# @example
#   @@maverick_ros::rosmaster { $instance_name:
#       ...
#   }
#
# @param active
#   If true, starts the maverick-rosmaster@[instance] service and enables at boot.
# @param port
#   The port that the rosmaster instance will be set to listen on.
#
define maverick_ros::rosmaster (
    Boolean $active = true,
    Integer $port = 11311,
) {

    # it can happen, that ROS 1 was not installed properly, or ROS 1 was not installed at all
    $service_file_path = '/etc/systemd/system/maverick-rosmaster@.service'
    if file($service_file_path, '/dev/null') == '' {
        warning("No rosmaster service file $service_file_path. Problem with ROS 1 installation. Not installing Ros master.")
        return()
    }

    file { "/srv/maverick/config/ros/rosmaster-${name}.conf":
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
            require     => [ Exec["maverick-systemctl-daemon-reload"], File[$service_file_path ] ]
        }
        # Punch some holes in the firewall for rosmaster
        if defined(Class["::maverick_security"]) {
            maverick_security::firewall::firerule { "rosmaster-${name}":
                ports       => ["${port}"],
                ips         => lookup("firewall_ips"),
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
