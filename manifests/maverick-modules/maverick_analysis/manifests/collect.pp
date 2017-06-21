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
    
    ### Collectd Plugins
    collectd::plugin::aggregation::aggregator {'cpu':
        plugin           => 'cpu',
        agg_type         => 'cpu',
        groupby          => ["Host", "TypeInstance",],
        calculateaverage => true,
    }
    class { 'collectd::plugin::contextswitch': }
    class { 'collectd::plugin::cpu':
        reportbystate => true,
        reportbycpu => true,
        valuespercentage => true,
    }
    class { 'collectd::plugin::cpufreq': }
    class { 'collectd::plugin::df':
        mountpoints    => [],
        fstypes        => ['nfs','tmpfs','autofs','gpfs','proc','devpts'],
        ignoreselected => true,
    }
    class { 'collectd::plugin::disk':
        disks          => ['/^dm/'],
        ignoreselected => true,
        udevnameattr   => 'DM_NAME',
    }
    class { 'collectd::plugin::interface':
        interfaces     => ['lo'],
        ignoreselected => true
    }
    class { 'collectd::plugin::ipmi':
        # ignore_selected           => true,
        # sensors                   => ['temperature'],
        notify_sensor_add         => true,
        notify_sensor_remove      => true,
        notify_sensor_not_present => true,
    }
    class { 'collectd::plugin::irq': }
    class { 'collectd::plugin::load': }
    class { 'collectd::plugin::memory': }
    class { 'collectd::plugin::processes': }
    class { 'collectd::plugin::protocols':
        values => ['/^Tcp:*/', '/^Udp:*/',],
        ignoreselected => false,
    }
    class { 'collectd::plugin::sensors': }
    class { 'collectd::plugin::swap':
        reportbydevice => false,
        reportbytes    => true
    }
    class { 'collectd::plugin::thermal': }
    class { 'collectd::plugin::uptime': }
    class { 'collectd::plugin::users': }
    # This collects a lot of data which we don't really need
    #class { 'collectd::plugin::vmem':
    #    verbose => true,
    #}

    # Configure an exec plugin to run power script on Joule platform, to retrieve power consumption
    if $joule_present == "yes" {
        collectd::plugin::exec::cmd { 'joule-power':
            user => mav,
            group => mav,
            exec => ["/srv/maverick/software/maverick/manifests/maverick-modules/maverick_analysis/files/joule-power.sh"],
        }
    }
    
    # Configure collect to send metrics to influxdb
    collectd::plugin::network::server{'localhost':
        port => 25826,
        securitylevel   => '',
    }

}