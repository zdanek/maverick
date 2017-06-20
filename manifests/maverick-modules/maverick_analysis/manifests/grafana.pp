class maverick_analysis::grafana (
    $webport = "6790",
) {
    
    ensure_packages(["sqlite3"])
    
    file { ["/srv/maverick/software/grafana", "/srv/maverick/data/analysis/grafana", "/srv/maverick/var/log/analysis/grafana"]:
        ensure      => directory,
        mode        => "755",
        owner       => "mav",
        group       => "mav",
    }
    
    # Source build, disabled for now
    if (1 == 2) {
        exec { "grafana-get":
            environment     => ["GOPATH=/srv/maverick/software/grafana"],
            command         => "/usr/bin/go get github.com/grafana/grafana",
            creates         => "/srv/maverick/software/grafana/src/github.com/grafana/grafana/README.md",
            user            => "mav",
            returns         => [0, 1], # This returns an error, ignore it
        } ->
        exec { "grafana-setup":
            environment     => ["GOPATH=/srv/maverick/software/grafana"],
            cwd             => "/srv/maverick/software/grafana/src/github.com/grafana/grafana",
            command         => "/usr/bin/go run build.go setup",
            user            => "mav",
        } ->
        exec { "grafana-build":
            environment     => ["GOPATH=/srv/maverick/software/grafana"],
            cwd             => "/srv/maverick/software/grafana/src/github.com/grafana/grafana",
            command         => "/usr/bin/go run build.go build",
            user            => "mav",
        }
    }
    
    file { "/etc/systemd/system/maverick-grafana.service":
        owner       => "root",
        group       => "root",
        mode        => "644",
        source      => "puppet:///modules/maverick_analysis/grafana.service",
        notify      => Exec["maverick-systemctl-daemon-reload"],
    } ->
    service_wrapper { "grafana-server":
        ensure      => "stopped",
        enable      => false,
    } ->
    class { "::grafana": 
        cfg_location        => "/srv/maverick/data/config/analysis/grafana.ini",
        cfg => {
            app_mode => 'production',
            server   => {
              http_port     => $webport,
            },
            users    => {
              allow_sign_up => false,
            },
      },
      data_dir              => "/srv/maverick/data/analysis/grafana",
      service_name          => "maverick-grafana",
      version               => "4.3.2",
    } ->
    # Create mav user in grafana
    #exec { "grafana-mavuser":
    #    command         => "/usr/sbin/grafana-cli ",
    #}
 
    grafana_datasource { 'influxdb':
        grafana_url       => "http://localhost:${webport}",
        grafana_user      => 'admin',
        grafana_password  => 'admin',
        type              => 'influxdb',
        url               => 'http://localhost:8086',
        database          => 'maverick',
        access_mode       => 'proxy',
        is_default        => true,
        require             => Service["maverick-grafana"],
    }

    grafana_dashboard { 'system_dashboard':
        grafana_url       => "http://localhost:${webport}",
        grafana_user      => 'admin',
        grafana_password  => 'admin',
        content           => template('maverick_analysis/grafana-system-dashboard.json'),
        require             => Service["maverick-grafana"],
    }

    if defined(Class["::maverick_security"]) {
        maverick_security::firewall::firerule { "grafana":
            ports       => $webport,
            ips         => lookup("firewall_ips"),
            proto       => "tcp"
        }
    }
    
}