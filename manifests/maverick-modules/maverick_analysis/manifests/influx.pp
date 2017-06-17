class maverick_analysis::influx (
    $source = "https://github.com/influxdata/influxdb.git",
    $branch = "v1.2.4",
    $active = true,
) {
    
    # Install Go
    ensure_packages(["golang"])

    # Install influx repo key
    exec { "influx-key":
        command         => "/usr/bin/curl -sL https://repos.influxdata.com/influxdb.key | sudo apt-key add -",
        unless          => "/usr/bin/apt-key list |/bin/egrep '2582\s?E0C5'",
    }
    
    # Install influx repo
    if $::operatingsystem == "Debian" {
        exec { "influx-repo":
            command         => "/bin/bash -c 'source /etc/os-release; test \$VERSION_ID = \"7\" && echo \"deb https://repos.influxdata.com/debian wheezy stable\" | sudo tee /etc/apt/sources.list.d/influxdb.list; test \$VERSION_ID = \"8\" && echo \"deb https://repos.influxdata.com/debian jessie stable\" | sudo tee /etc/apt/sources.list.d/influxdb.list'",
            notify          => Exec["maverick-aptget-update"],
            creates         => "/etc/apt/sources.list.d/influxdb.list",
        }
    } elsif $::operatingsystem == "Ubuntu" {
        exec { "influx-repo":
            command         => "/bin/bash -c 'source /etc/lsb-release; echo \"deb https://repos.influxdata.com/\${DISTRIB_ID,,} \${DISTRIB_CODENAME} stable\" | sudo tee /etc/apt/sources.list.d/influxdb.list'",
            notify          => Exec["maverick-aptget-update"],
            creates         => "/etc/apt/sources.list.d/influxdb.list",
        }
    }

    # Install influxdb
    package { "influxdb":
        ensure     => installed,
        require     => Exec["influx-repo"],
    } ->
    file { "/srv/maverick/data/config/analysis/influxdb.conf":
        content     => template("maverick_analysis/influxdb.conf.erb"),
        owner       => "root",
        group       => "root",
    } ->
    file { ["/srv/maverick/data/analysis/influxdb", "/srv/maverick/var/lib/influxdb", "/srv/maverick/var/lib/influxdb/meta", "/srv/maverick/var/lib/influxdb/wal", "/srv/maverick/var/log/analysis"]:
        owner       => "mav",
        group       => "mav",
        ensure      => directory,
        mode        => "755",
    } ->
    file { "/etc/systemd/system/maverick-influxd.service":
        source      => "puppet:///modules/maverick_analysis/influxd.service",
        owner       => "root",
        group       => "root",
        mode        => "644",
        notify      => Exec["maverick-systemctl-daemon-reload"],
    } ->
    # Ensure system influxd instance is stopped
    service_wrapper { "influxdb":
        ensure      => stopped,
        enable      => false,
    }
    
    if $active == true {
        service_wrapper { "maverick-influxd":
            ensure      => running,
            enable      => true,
            require     => File["/etc/systemd/system/maverick-influxd.service"],
        }
    } else {
        service_wrapper { "maverick-influxd":
            ensure      => stopped,
            enable      => false,
            require     => File["/etc/systemd/system/maverick-influxd.service"],
        }
    }
    
    # Ensure maverick metrics db exists
    exec { "influx-maverickdb":
        command         => "/usr/bin/influx -execute 'create database maverick'",
        unless          => "/usr/bin/influx -execute 'show databases' |grep maverick",
        user            => "mav",
        require         => Service_wrapper["maverick-influxd"],
    }
    
}