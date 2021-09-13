# @summary
#   Maverick_analysis::Influx class
#   This class installs/manages InfluxDB software (www.influxdata.com), which is used to store time series metrics, including system and flight controller data.
#
# @example Declaring the class
#   This class is included from maverick_analysis class and should not be included from elsewhere
#
# @param active
#   If true, set the maverick-influxd service to running and enabled (at boot).
# @param http_port
#   Port for influx to listen on for http endpoint
# @param collectd_listener_port
#   Port to listen on for collectd to deliver metrics
#
class maverick_analysis::influx (
    Boolean $active = true,
    Integer $http_port = 6020,
    Integer $collectd_listener_port = 6021,
) {

    # Install Go
    ensure_packages(["golang", "curl"])

    # Install influx repo
    if $::operatingsystem == "Debian" or $::operatingsystem == "Raspbian" {
        if $::operatingsystemmajrelease == "7" {
            $_influx_command = "/bin/echo \"deb https://repos.influxdata.com/debian wheezy stable\" | sudo tee /etc/apt/sources.list.d/influxdb.list"
        } elsif $::operatingsystemmajrelease == "8" {
            $_influx_command = "/bin/echo \"deb https://repos.influxdata.com/debian jessie stable\" | sudo tee /etc/apt/sources.list.d/influxdb.list"
        } elsif $::operatingsystemmajrelease == "9" {
            $_influx_command = "/bin/echo \"deb https://repos.influxdata.com/debian stretch stable\" | sudo tee /etc/apt/sources.list.d/influxdb.list"
        } elsif $::operatingsystemmajrelease == "10" {
            $_influx_command = "/bin/echo \"deb https://repos.influxdata.com/debian buster stable\" | sudo tee /etc/apt/sources.list.d/influxdb.list"
        }
    } elsif $::operatingsystem == "Ubuntu" {
        if $::operatingsystem == "Ubuntu" and versioncmp($::operatingsystemmajrelease, "18") >= 0 {
            $_influx_command = "/bin/bash -c 'source /etc/lsb-release; echo \"deb https://repos.influxdata.com/\${DISTRIB_ID,,} artful stable\" | sudo tee /etc/apt/sources.list.d/influxdb.list'"
        } else {
            $_influx_command = "/bin/bash -c 'source /etc/lsb-release; echo \"deb https://repos.influxdata.com/\${DISTRIB_ID,,} \${DISTRIB_CODENAME} stable\" | sudo tee /etc/apt/sources.list.d/influxdb.list'"
        }
    }

    # Install influx repo key
    exec { "influx-key":
        command         => "/usr/bin/curl -sL https://repos.influxdata.com/influxdb.key | sudo apt-key add -",
        creates         => "/etc/apt/sources.list.d/influxdb.list",
    } ->
    exec { "influx-repo":
        command         => $_influx_command,
        creates         => "/etc/apt/sources.list.d/influxdb.list",
        notify          => Exec["apt_update"],
    }
    # Install influxdb
    package { "influxdb":
        ensure      => latest,
        require     => [ Exec["influx-repo"], Exec["apt_update"] ],
    } ->
    file { "/srv/maverick/config/analysis/influxdb.conf":
        content     => template("maverick_analysis/influxdb.conf.erb"),
        owner       => "mav",
        group       => "mav",
        notify      => Service["maverick-influxd"],
    } ->
    file { ["/srv/maverick/data/analysis/influxdb", "/srv/maverick/data/analysis/influxdb/meta", "/srv/maverick/data/analysis/influxdb/wal", "/srv/maverick/data/analysis/influxdb/data", "/srv/maverick/var/log/analysis"]:
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
    service { "influxdb":
        ensure      => stopped,
        enable      => false,
    }

    if $active == true {
        exec { "influxd-systemd-activate":
            command     => "/bin/systemctl daemon-reload",
            unless      => "/bin/systemctl list-units |grep maverick-influxd",
        } ->
        service { "maverick-influxd":
            ensure      => running,
            enable      => true,
            require     => [ Service["influxdb"], Class["maverick_analysis::collect"] ],
        }
    } else {
        service { "maverick-influxd":
            ensure      => stopped,
            enable      => false,
            require     => [ Service["influxdb"], Class["maverick_analysis::collect"] ],
        }
    }

    # Install python library
    install_python_module { "pip-influxdb":
        ensure          => atleast,
        version         => "5.0.0",
        pkgname         => "influxdb",
    }

    # Configure collect to send metrics to influxdb
    collectd::plugin::network::server{'localhost':
        port            => $collectd_listener_port,
        securitylevel   => "None",
        require         => Service["maverick-influxd"],
    }

    # status.d entry
    file { "/srv/maverick/software/maverick/bin/status.d/121.analysis/102.influx.status":
        owner   => "mav",
        content => "influxd,TimeSeries Database\n",
    }
}
