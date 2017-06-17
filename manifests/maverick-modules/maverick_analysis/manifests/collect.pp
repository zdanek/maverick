class maverick_analysis::collect (
    $active = true,
) {
    
    # Collectd repos only provide i386/amd64 packages
    if $::architecture == "i386" or $::architecture == "amd64" {
        $manage_repo = true
    } else {
        $manage_repo = false
    }

    class { "collectd":
        purge           => true,
        recurse         => true,
        purge_config    => true,
        minimum_version => '5.4',
        manage_repo     => $manage_repo,
        require         => Service_wrapper["maverick-influxd"],
    }
    
    class { 'collectd::plugin::cpu':
        reportbystate => true,
        reportbycpu => true,
        valuespercentage => true,
    }

    class { 'collectd::plugin::cpufreq':
    }
    
    class { 'collectd::plugin::load':
    }

    class { 'collectd::plugin::memory':
    }
    
    class { 'collectd::plugin::swap':
        reportbydevice => false,
        reportbytes    => true
    }

    class {'collectd::plugin::uptime':
    }

    class { 'collectd::plugin::vmem':
        verbose => true,
    }
    
    # Configure collect to send metrics to influxdb
    collectd::plugin::network::server{'localhost':
        port => 25826,
        securitylevel   => '',
    }

}