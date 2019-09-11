class maverick_analysis (
    $influxdb = true,
    $collectd = true,
    $grafana = true,
    $mavlogd = true,
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
