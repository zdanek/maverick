define maverick_web::api (
    $instance = "fc",
    $active = true,
    $apiport = 6800,
    $rosport = 11311,
    $server_hostname = $maverick_web::server_fqdn,
) {
    # This class creates an instance of maverick-api.
    # The actual installation of the maverick-api and dependencies happens in maverick_web::maverick_api

    file { "/srv/maverick/var/log/web/api/${instance}":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    } ->
    file { "/srv/maverick/config/web/api-${instance}.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        mode        => "644",
        content     => template("maverick_web/api.conf.erb"),
        notify      => Service_wrapper["maverick-api@${instance}"],
    } ->
    file { "/srv/maverick/config/web/api-${instance}.json":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        mode        => "644",
        content     => template("maverick_web/api.json.erb"),
        notify      => Service_wrapper["maverick-api@${instance}"],
    } 

    if $active == true {
        service_wrapper { "maverick-api@${instance}":
            ensure      => running,
            enable      => true,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-api@.service"] ]
        }
    } else {
        service_wrapper { "maverick-api@${instance}":
            ensure      => stopped,
            enable      => false,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-api@.service"] ]
        }
    }

    nginx::resource::location { "maverick-api-${instance}":
        ssl                     => true,
        location                => "/web/api/${instance}/",
        proxy                   => "http://localhost:${apiport}/",
        server                  => $server_hostname,
        require                 => [ Class["maverick_web::maverick_api"], Class["maverick_gcs::fcs"], Class["nginx"] ],
    	proxy_connect_timeout   => "7d",
    	proxy_read_timeout      => "7d",
        proxy_set_header        => ['Upgrade $http_upgrade', 'Connection "upgrade"', 'Host $host', 'X-Real-IP $remote_addr', 'X-Forwarded-For $proxy_add_x_forwarded_for', 'Proxy ""'],
    	proxy_http_version      => "1.1",
    }
    
    if defined(Class["::maverick_security"]) {
        maverick_security::firewall::firerule { "maverick-api@${instance}":
            ports       => $apiport,
            ips         => lookup("firewall_ips"),
            proto       => "tcp"
        }
    }
    
}