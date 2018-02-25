class maverick_web::maverick_web (
    $webport = 6794,
    $active = true,
    $webpath_dev = '/dev/maverick/',
    $webpath_prod = '/web/maverick',
    $server_hostname = $maverick_web::server_fqdn,
    $auth_message = undef,
    $auth_file = undef,
) {
    
    oncevcsrepo { "git-maverick-web":
        gitsource   => "https://github.com/goodrobots/maverick-web.git",
        dest        => "/srv/maverick/code/maverick-web",
        revision    => "master",
        depth       => undef,
    } ->
    /*
    nodejs::npm { 'npm-maverick-web':
      user             => 'mav',
      home_dir         => '/srv/maverick',
      ensure           => 'present',
      target           => '/srv/maverick/code/maverick-web',
      use_package_json => true,
    } ->
    */
    exec { "npm-maverick-web":
        command     => "/usr/bin/npm install",
        cwd         => "/srv/maverick/code/maverick-web",
        creates     => "/srv/maverick/code/maverick-web/node_modules",
    } ->
    file { "/etc/systemd/system/maverick-web.service":
        owner       => "root",
        group       => "root",
        mode        => "644",
        source      => "puppet:///modules/maverick_web/maverick-web.service",
        notify      => Exec["maverick-systemctl-daemon-reload"],
    } -> 
    nginx::resource::location { "maverick-web":
        location    => $webpath_dev,
        proxy       => "http://localhost:${webport}/",
        server      => $server_hostname,
        auth_basic  => $auth_message,
        auth_basic_user_file => $auth_file,
        require     => [ Class["maverick_gcs::fcs"], Class["nginx"] ],
    } ->
    nginx::resource::location { "maverick-web-prod":
        location    => $webpath_prod,
        ensure          => present,
        ssl             => true,
        location_alias  => "/srv/maverick/code/maverick-web/dist",
        index_files     => ["index.html"],
        server          => $server_hostname,
        require         => [ Class["maverick_gcs::fcs"], Class["nginx"] ],
    }
    /*
    nginx::resource::location { "maverick-web-prod-precache":
        location    => "~ (index.html|service-worker.js)$",
        raw_append  => [
            "add_header Last-Modified \$date_gmt;",
            "add_header Cache-Control 'no-store, no-cache, must-revalidate, proxy-revalidate, max-age=0';",
            "if_modified_since off;",
            "expires off;",
            "etag off;"
        ],
        index_files     => [],
        server          => $server_hostname,
        ssl             => true,
        location_alias  => "/srv/maverick/code/maverick-web/dist/",
    }
    */
    
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