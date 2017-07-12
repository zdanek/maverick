class maverick_analysis (
    $influxdb = true,
    $collectd = true,
    $grafana = true,
    $mavlogd = true,
) {
    
    file { ["/srv/maverick/data/analysis", "/srv/maverick/data/config/analysis"]:
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
        class { "maverick_analysis::grafana":
            require     => [ Class["maverick_analysis::collect"], Class["maverick_analysis::influx"] ],
        }
    }
    
    if $mavlogd == true {
        class { "maverick_analysis::mavlogd":
            require     => [ Class["maverick_analysis::collect"], Class["maverick_analysis::influx"], Class["maverick_analysis::grafana"] ],
        }
    }
    
}