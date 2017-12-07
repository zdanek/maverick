class maverick_analysis::grafana (
    $webport = "6790",
    $rootpath = "/analysis/grafana/",
    $grafana_version = latest,
    $grafana_raspberry_version = "4.6.1",
    $grafana_firewall_rules = false,
    $mav_password = 'e35f84e5859dfe5dfe2a9f6ed2086884c3a5e41d206c6e704b48cf45a0dda574ad85b4e9362e8d89eee3eb82e7ef34528ea4',
    $mav_salt = 'ry48G1ZHyi',
    $admin_hash = '0a66a2e243118cc863cbcaa5c1bae486a57a008e5b17b68f5c00cabd7dcc6b6974af6806348d75f17fb709762cda2038c05c',
    $admin_password = 'theneedforspeed',
    $admin_salt = 'F3FAxVm33R',
) {
    
    ensure_packages(["sqlite3"])

    file { ["/srv/maverick/data/analysis/grafana", "/srv/maverick/var/log/analysis/grafana"]:
        ensure      => directory,
        mode        => "755",
        owner       => "mav",
        group       => "mav",
    }
    file { "/srv/maverick/software/grafana":
        ensure      => absent,
        force       => true,
    }

    if $joule_present == "yes" {
        $_dashboard = "system-dashboard-joule.json"
    } elsif $raspberry_present == "yes" {
        $_dashboard = "system-dashboard-raspberry.json"
    } else {
        $_dashboard = "system-dashboard-generic.json"
    }

    if $raspberry_present == "yes" {
        $manage_package_repo = false
        # Don't use autodetect above, because we want OS images to be compatible with Zero
        $package_source = "https://dl.bintray.com/fg2it/deb-rpi-1b/main/g/grafana_${grafana_raspberry_version}_armhf.deb"
        $install_method = "package"
    } else {
        $manage_package_repo = true
        $package_source = undef
        $install_method = "repo"
    }
    file { "/etc/systemd/system/maverick-grafana.service":
        owner       => "root",
        group       => "root",
        mode        => "644",
        source      => "puppet:///modules/maverick_analysis/grafana.service",
        notify      => Exec["maverick-systemctl-daemon-reload"],
    } ->
    class { "::grafana": 
        cfg_location        => "/srv/maverick/config/analysis/grafana.ini",
        cfg => {
            app_mode => 'production',
            paths   => {
                data        => "/srv/maverick/data/analysis/grafana",
            },
            server   => {
              http_port     => $webport,
              root_url      => "%(protocol)s://%(domain)s:${rootpath}",
            },
            users    => {
              allow_sign_up => false,
            },
      },
      data_dir              => "/srv/maverick/data/analysis/grafana",
      install_method        => $install_method,
      manage_package_repo   => $manage_package_repo,
      package_source        => $package_source,
      service_name          => "maverick-grafana",
      version               => $grafana_version,
    } ->
    service_wrapper { "grafana-server":
        ensure      => "stopped",
        enable      => false,
    } ->
    http_conn_validator { 'grafana-postdelay' :
        host    => '127.0.0.1',
        port    => $webport,
        use_ssl => false,
        test_url => '/public/img/grafana_icon.svg',
    } ->
    # Create maverick org in grafana
    exec { "grafana-maverickorg":
        unless          => "/usr/bin/sqlite3 /srv/maverick/data/analysis/grafana/grafana.db 'select * from main.org' |grep Maverick",
        command         => "/bin/sleep 10; /usr/bin/sqlite3 /srv/maverick/data/analysis/grafana/grafana.db \"insert into main.org values(10,0,'Maverick','','','','','','','','2017-06-20 11:02:55','2017-06-20 11:15:51')\"", # sleep is to give grafana enough time to fire up and release the db
        user            => "mav",
    } ->
    # Delete old mav user
    exec { "grafana-deloldmavuser":
        onlyif          => "/usr/bin/sqlite3 /srv/maverick/data/analysis/grafana/grafana.db 'select * from main.user where id=10' |grep mav",
        command         => "/usr/bin/sqlite3 /srv/maverick/data/analysis/grafana/grafana.db \"delete from main.user where id=10\"",
        user            => "mav",
    } ->
    # Create mav user in grafana
    exec { "grafana-mavuser":
        unless          => "/usr/bin/sqlite3 /srv/maverick/data/analysis/grafana/grafana.db 'select * from main.user' |grep mav",
        command         => "/usr/bin/sqlite3 /srv/maverick/data/analysis/grafana/grafana.db \"insert into main.user values(100,0,'mav','mav','Maverick User','${mav_password}','{$mav_salt}','yICOZzT82L','',10,0,0,'','2017-06-21 12:54:43','2017-06-21 12:54:43',1,'2017-06-21 12:54:43')\"",
        user            => "mav",
    } ->
    # Link mav user to org
    exec { "grafana-linkmav":
        unless          => "/usr/bin/sqlite3 /srv/maverick/data/analysis/grafana/grafana.db 'select * from main.org_user where org_id=\"10\" and user_id=\"100\"' |grep Viewer",
        command         => "/usr/bin/sqlite3 /srv/maverick/data/analysis/grafana/grafana.db \"insert into main.org_user values('100','10','100','Viewer','2017-06-21 13:43:38','2017-06-21 13:43:38')\"",
        user            => "mav",
    } ->
    # Create admin user in grafana
    exec { "grafana-adminuser":
        unless          => "/usr/bin/sqlite3 /srv/maverick/data/analysis/grafana/grafana.db 'select * from main.user' |grep admin",
        command         => "/usr/bin/sqlite3 /srv/maverick/data/analysis/grafana/grafana.db \"insert into main.user values(101,0,'admin','admin','Maverick Admin','${admin_hash}','${admin_salt}','yICOZzT82L','',10,0,0,'','2017-06-21 12:54:43','2017-06-21 12:54:43',1,'2017-06-21 12:54:43')\"",
        user            => "mav",
    } ->
    # Link admin user to org
    exec { "grafana-linkadmin":
        unless          => "/usr/bin/sqlite3 /srv/maverick/data/analysis/grafana/grafana.db 'select * from main.org_user where org_id=\"10\" and user_id=\"101\"' |grep Admin",
        command         => "/usr/bin/sqlite3 /srv/maverick/data/analysis/grafana/grafana.db \"insert into main.org_user values('101','10','101','Admin','2017-06-21 13:43:38','2017-06-21 13:43:38')\"",
        user            => "mav",
    } ->
    grafana_datasource { 'influxdb':
        grafana_url       => "http://localhost:${webport}",
        grafana_user      => 'admin',
        grafana_password  => $admin_password,
        type              => 'influxdb',
        url               => 'http://localhost:8086',
        database          => 'maverick',
        access_mode       => 'proxy',
        is_default        => true,
        require             => [ Service["maverick-grafana"], Http_conn_validator["grafana-postdelay"] ],
    } ->
    grafana_dashboard { 'system_dashboard':
        title               => "System Dashboard",
        grafana_url       => "http://localhost:${webport}",
        grafana_user      => 'admin',
        grafana_password  => $admin_password,
        content           => template("maverick_analysis/${_dashboard}"),
        require             => [ Service["maverick-grafana"], Http_conn_validator["grafana-postdelay"] ],
    } ->
    grafana_dashboard { 'flight_dashboard':
        title               => "Flight Data Analysis",
        grafana_url       => "http://localhost:${webport}",
        grafana_user      => 'admin',
        grafana_password  => $admin_password,
        content           => template("maverick_analysis/flight-dashboard-ardupilot.json"),
        require             => [ Service["maverick-grafana"], Http_conn_validator["grafana-postdelay"] ],
    } ->
    grafana_dashboard { 'ekf2_dashboard':
        title               => "Flight EKF2 Analysis",
        grafana_url       => "http://localhost:${webport}",
        grafana_user      => 'admin',
        grafana_password  => $admin_password,
        content           => template("maverick_analysis/flight-ekf2-ardupilot.json"),
        require             => [ Service["maverick-grafana"], Http_conn_validator["grafana-postdelay"] ],
    } ->
    grafana_dashboard { 'ekf3_dashboard':
        title               => "Flight EKF3 Analysis",
        grafana_url       => "http://localhost:${webport}",
        grafana_user      => 'admin',
        grafana_password  => $admin_password,
        content           => template("maverick_analysis/flight-ekf3-ardupilot.json"),
        require             => [ Service["maverick-grafana"], Http_conn_validator["grafana-postdelay"] ],
    } ->
    grafana_dashboard { 'ekf2ekf3_dashboard':
        title               => "Flight EKF2-EKF3 Analysis",
        grafana_url       => "http://localhost:${webport}",
        grafana_user      => 'admin',
        grafana_password  => $admin_password,
        content           => template("maverick_analysis/flight-ekf2ekf3-ardupilot.json"),
        require             => [ Service["maverick-grafana"], Http_conn_validator["grafana-postdelay"] ],
    } ->
    grafana_dashboard { 'mavexplorer_mavgraphs_dashboard':
        title               => "MAVExplorer Mavgraphs",
        grafana_url       => "http://localhost:${webport}",
        grafana_user      => 'admin',
        grafana_password  => $admin_password,
        content           => template("maverick_analysis/mavexplorer-mavgraphs.json"),
        require             => [ Service["maverick-grafana"], Http_conn_validator["grafana-postdelay"] ],
    }
    
    if defined(Class["::maverick_web"]) {
        nginx::resource::location { "web-analysis-graphs":
            location    => "/analysis/grafana/",
            proxy       => 'http://localhost:6790/',
            server      => "${::hostname}.local",
            require     => [ Class["maverick_gcs::fcs"], Class["nginx"] ],
        }
    }

    if $grafana_firewall_rules == true {
        if defined(Class["::maverick_security"]) {
            maverick_security::firewall::firerule { "grafana":
                ports       => $webport,
                ips         => lookup("firewall_ips"),
                proto       => "tcp"
            }
        }
    }
    
}
