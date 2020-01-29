# @summary
#   Maverick_analysis class
#   This class controls all other classes in maverick_analysis module.
#
# @example Declaring the class
#   This class is included from the environment manifests and is not usually included elsewhere.
#   It could be included selectively from eg. minimal environment.
#
# @param influxdb
#   Whether to include the maverick_analysis::influxdb class.  Note this doesn't activate influxdb itself, just includes the class.
# @param collectd
#   Whether to include the maverick_analysis::collectd class.  Note this doesn't activate collectd itself, just includes the class.
# @param collectd
#   Whether to include the maverick_analysis::grafana class.  Note this doesn't activate grafana itself, just includes the class.
# @param collectd
#   Whether to include the maverick_analysis::mavlogd class.  Note this doesn't activate mavlogd itself, just includes the class.
#
class maverick_analysis (
    Boolean $influxdb = true,
    Boolean $collectd = true,
    Boolean $grafana = true,
    Boolean $mavlogd = true,
) {
    
    # Create status.d directory for maverick status`
    file { "/srv/maverick/software/maverick/bin/status.d/121.analysis":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    } ->
    file { "/srv/maverick/software/maverick/bin/status.d/121.analysis/__init__":
        owner       => "mav",
        content     => "Analysis Services",
    }

    file { ["/srv/maverick/data/analysis", "/srv/maverick/config/analysis"]:
        owner       => "mav",
        group       => "mav",
        mode        => "755",
        ensure      => directory,
    }

    if $collectd == true {
        # Note class named collect instead of collectd to not conflict with other classes
        class { "maverick_analysis::collect": }
    }
    
    if $influxdb == true {
        # Note class named influx instead of influxdb to not conflict with other classes
        if defined(Class["maverick_analysis::collect"]) {
            class { "maverick_analysis::influx": }
        } else {
            class { "maverick_analysis::influx":
                require     => Class["maverick_analysis::collect"],
            }
        }
    }
    
    if $grafana == true {
        if defined(Class["mavlogd"]) {
            $_before = Class["mavlogd"]
        } else {
            $_before = undef
        }
        class { "maverick_analysis::grafana":
            require     => Class["maverick_analysis::collect"],
            before      => $_before,
        }
    }
    
    if $mavlogd == true {
        class { "maverick_analysis::mavlogd":
            require     => [ Class["maverick_analysis::collect"], Class["maverick_analysis::influx"], ],
        }
    }
    
}
