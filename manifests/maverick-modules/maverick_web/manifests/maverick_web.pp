class maverick_web::maverick_web (
    $webport = 6794,
    $active = false,
    $webpath_dev = '/dev/maverick/',
    $webpath_prod = '/web/maverick',
    $server_hostname = $maverick_web::server_fqdn,
    $auth_message = undef,
    $auth_file = undef,
) {
    
    # Install node dependency globally, easier cross-platform
    ensure_packages(["phantomjs"])
    
    # Install dev repo, register maverick-webdev service
    package { '@vue/cli':
        ensure   => 'present',
        provider => 'npm',
    } ->
    oncevcsrepo { "git-maverick-web":
        gitsource   => "https://github.com/goodrobots/maverick-web.git",
        dest        => "/srv/maverick/code/maverick-web",
        revision    => "master",
        depth       => undef,
    } ->
    exec { "npm-maverick-web":
        command     => "/usr/bin/npm install",
        cwd         => "/srv/maverick/code/maverick-web",
        creates     => "/srv/maverick/code/maverick-web/node_modules",
        user        => "mav",
        environment => ["QT_QPA_PLATFORM=offscreen"], # Fix to allow global phantomjs to run headless
        timeout     => 0,
        require     => [ Package["phantomjs"], Class["maverick_web::nodejs"], Package["nodejs"], ],
    } ->
    file { "/etc/systemd/system/maverick-webdev.service":
        owner       => "root",
        group       => "root",
        mode        => "644",
        source      => "puppet:///modules/maverick_web/maverick-webdev.service",
        notify      => Exec["maverick-systemctl-daemon-reload"],
    } -> 
    nginx::resource::location { "maverick-webdev":
        location    => $webpath_dev,
        ensure      => present,
        ssl         => true,
        proxy       => "http://localhost:${webport}/",
        server      => $server_hostname,
        auth_basic  => $auth_message,
        auth_basic_user_file => $auth_file,
        require     => [ Class["maverick_gcs::fcs"], Class["nginx"] ],
    }
    
    # Install prod repo, register nginx location
    oncevcsrepo { "git-maverick-web-dist":
        gitsource   => "https://github.com/goodrobots/maverick-web-dist.git",
        dest        => "/srv/maverick/software/maverick-web",
        revision    => "master",
        depth       => undef,
    } ->
    nginx::resource::location { "maverick-web-prod":
        location        => $webpath_prod,
        ensure          => present,
        ssl             => true,
        location_alias  => "/srv/maverick/software/maverick-web",
        index_files     => ["index.html"],
        server          => $server_hostname,
        auth_basic      => $auth_message,
        auth_basic_user_file => $auth_file,
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
        location_alias  => "/srv/maverick/software/maverick-web/",
    }
    */

    # Stop old web service if it exists
    service { 'maverick':
        ensure  => stopped,
        enable  => false,
    }
    
    # Bring webdev service to desired state
    if $active == true {
        service { "maverick-webdev":
            ensure      => running,
            enable      => true,
            require     => Exec["maverick-systemctl-daemon-reload"],
        }
    } else {
        service { "maverick-webdev":
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