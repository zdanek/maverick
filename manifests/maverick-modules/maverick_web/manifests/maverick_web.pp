class maverick_web::maverick_web (
    $webport = 6794,
    $active = true,
    $server_hostname = $maverick_web::server_fqdn,
) {
    
    oncevcsrepo { "git-maverick-web":
        gitsource   => "https://github.com/goodrobots/maverick-web.git",
        dest        => "/srv/maverick/code/maverick-web",
        revision    => "master",
        depth       => undef,
    } ->
    nodejs::npm { 'npm-maverick-web':
      user             => 'mav',
      home_dir         => '/srv/maverick',
      ensure           => 'present',
      target           => '/srv/maverick/code/maverick-web',
      use_package_json => true,
    } ->
    file { "/etc/systemd/system/maverick-web.service":
        owner       => "root",
        group       => "root",
        mode        => "644",
        source      => "puppet:///modules/maverick_web/maverick-web.service",
        notify      => Exec["maverick-systemctl-daemon-reload"],
    } -> 
    nginx::resource::location { "maverick-web":
        location    => "/maverick-web/",
        proxy       => "http://localhost:${webport}/",
        server      => $server_hostname,
        require     => [ Class["maverick_gcs::fcs"], Class["nginx"] ],
    } ->
    nginx::resource::location { "maverick-web-prod":
        location    => "/web/maverick",
        ensure          => present,
        ssl             => true,
        location_alias  => "/srv/maverick/code/maverick-web/dist",
        index_files     => ["index.html"],
        server          => $server_hostname,
        require         => [ Class["maverick_gcs::fcs"], Class["nginx"] ],
    }

    if $active == true {
        service { "maverick-web":
            ensure      => running,
            enable      => true,
            require     => Exec["maverick-systemctl-daemon-reload"],
        }
    } else {
        service { "maverick-web":
            ensure      => stopped,
            enable      => false,
            require     => Exec["maverick-systemctl-daemon-reload"],
        }
    }
    
    if defined(Class["::maverick_security"]) {
        maverick_security::firewall::firerule { "maverick-web":
            ports       => $webport,
            ips         => lookup("firewall_ips"),
            proto       => "tcp"
        }
    }

}