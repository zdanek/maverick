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
    
    if $joule_present == "yes" {
        $_dashboard = "system-dashboard-joule.json"
    } elsif $raspberry_present == "yes" {
        $_dashboard = "system-dashboard-raspberry.json"
    } else {
        $_dashboard = "system-dashboard-generic.json"
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
    # Create maverick org in grafana
    exec { "grafana-maverickorg":
        unless          => "/usr/bin/sqlite3 /srv/maverick/data/analysis/grafana/grafana.db 'select * from main.org' |grep Maverick",
        command         => "/bin/sleep 10; /usr/bin/sqlite3 /srv/maverick/data/analysis/grafana/grafana.db \"insert into main.org values(10,0,'Maverick','','','','','','','','2017-06-20 11:02:55','2017-06-20 11:15:51')\"", # sleep is to give grafana enough time to fire up and release the db
        user            => "mav",
    } ->
    # Create mav user in grafana
    exec { "grafana-mavuser":
        unless          => "/usr/bin/sqlite3 /srv/maverick/data/analysis/grafana/grafana.db 'select * from main.user' |grep mav",
        command         => "/usr/bin/sqlite3 /srv/maverick/data/analysis/grafana/grafana.db \"insert into main.user values(10,0,'mav','mav','Maverick User','e35f84e5859dfe5dfe2a9f6ed2086884c3a5e41d206c6e704b48cf45a0dda574ad85b4e9362e8d89eee3eb82e7ef34528ea4','ry48G1ZHyi','yICOZzT82L','',10,0,0,'','2017-06-21 12:54:43','2017-06-21 12:54:43',1)\"",
        user            => "mav",
    } ->
    # Link mav user to org
    exec { "grafana-linkorg":
        unless          => "/usr/bin/sqlite3 /srv/maverick/data/analysis/grafana/grafana.db 'select * from main.org_user where org_id=\"10\" and user_id=\"10\"' |grep Admin",
        command         => "/usr/bin/sqlite3 /srv/maverick/data/analysis/grafana/grafana.db \"insert into main.org_user values('10','10','10','Admin','2017-06-21 13:43:38','2017-06-21 13:43:38')\"",
        user            => "mav",
    } ->
    grafana_datasource { 'influxdb':
        grafana_url       => "http://localhost:${webport}",
        grafana_user      => 'mav',
        grafana_password  => 'wingman',
        type              => 'influxdb',
        url               => 'http://localhost:8086',
        database          => 'maverick',
        access_mode       => 'proxy',
        is_default        => true,
        require             => Service["maverick-grafana"],
    } ->
    grafana_dashboard { 'system_dashboard':
        title               => "System Dashboard",
        grafana_url       => "http://localhost:${webport}",
        grafana_user      => 'mav',
        grafana_password  => 'wingman',
        content           => template("maverick_analysis/${_dashboard}"),
        require             => Service["maverick-grafana"],
    } ->
    grafana_dashboard { 'flight_dashboard':
        title               => "Flight Data Analysis",
        grafana_url       => "http://localhost:${webport}",
        grafana_user      => 'mav',
        grafana_password  => 'wingman',
        content           => template("maverick_analysis/flight-dashboard-ardupilot.json"),
        require             => Service["maverick-grafana"],
    } ->
    grafana_dashboard { 'ekf2_dashboard':
        title               => "Flight EKF2 Analysis",
        grafana_url       => "http://localhost:${webport}",
        grafana_user      => 'mav',
        grafana_password  => 'wingman',
        content           => template("maverick_analysis/flight-ekf2-ardupilot.json"),
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