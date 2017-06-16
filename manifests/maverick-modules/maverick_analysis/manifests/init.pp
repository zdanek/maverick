class maverick_analysis (
) {
    
    file { ["/srv/maverick/data/analysis", "/srv/maverick/data/config/analysis"]:
        owner       => "mav",
        group       => "mav",
        mode        => "755",
        ensure      => directory,
    }

    class { "maverick_analysis::influxdb": }
    
}