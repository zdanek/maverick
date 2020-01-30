# @summary
#   This function creates a mavros instance.  It is typically called by modules that also create mavlink proxy and rosmaster instances.
#
# @example
#   @@maverick_ros::mavros { $instance_name:
#       ...
#   }
#
# @param active
#   If true, starts the maverick-mavros@[instance] service and enables at boot.
# @param rosmaster_port
#   The rosmaster port that this instance of mavros connects to.
# @param mavlink_port
#   The mavlink port that this instance of mavros connects to.
# @param mavros_startup_delay
#   This is a delay fudge for slower boards to ensure rosmaster and mavlink proxies have had time to fully start.
# @param mavros_launcher
#   The ros/mavros launcher to use to start mavros.
# @param source_sysid
#   If set, define the mavlink source system ID.
# @param source_cmpid
#   If set, define the mavlink source component ID.
# @param target_sysid
#   If set, define the mavlink target system ID.
# @param target_cmpid
#   If set, define the mavlink target component ID.
#
define maverick_ros::mavros (
    Boolean $active = true,
    Integer $rosmaster_port = 11311,
    Integer $mavlink_port = 5770,
    Integer $mavros_startup_delay = 10,
    String $mavros_launcher = "apm.launch",
    Optional[Integer] $source_sysid = undef,
    Optional[Integer] $source_cmpid = undef,
    Optional[Integer] $target_sysid = undef,
    Optional[Integer] $target_cmpid = undef,
) {
    
    file { "/srv/maverick/config/ros/mavros-${name}.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        mode        => "644",
        content     => template("maverick_ros/mavros.conf.erb"),
        notify      => Service["maverick-mavros@${name}"],
    }
    
    if $active == true {
    	service { "maverick-mavros@${name}":
            ensure      => running,
            enable      => true,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-mavros@.service"] ]
        }
    } else {
    	service { "maverick-mavros@${name}":
            ensure      => stopped,
            enable      => false,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-mavros@.service"] ]
        }
    }
    
}
