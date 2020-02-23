# @summary
#   This function creates a maverick-api instance.  It is typically called by modules that also create mavlink proxy and rosmaster instances.
#
# @example
#   @@maverick_web::api { $instance_name:
#       ...
#   }
#
# @param active
#   If true, start the maverick-api@[xxx] service and enable at boot time.
# @param instance
#   Name of the -api instance.  This must be unique.
# @param apiport
#   TCP port for the api to listen on.
# @param rosport
#   ROS rosmaster port for -api instance to connect to.
# @param server_hostname
#   Webserver vhost to use for reverse proxy.
# @param devmode
#   If true, activate -api development mode.
# @param debug
#   If true, activet -api debug mode.
# @param replaceconfig
#   If true, fully manage the -api config and overwrite with values from parameters set here.
#
define maverick_web::api (
    Boolean $active = true,
    String $instance = "fc",
    Integer $apiport = 6800,
    Integer $rosport = 11311,
    String $server_hostname = $maverick_web::server_fqdn,
    Boolean $devmode = false,
    Boolean $debug = false,
    Boolean $replaceconfig = true,
) {
    # This class creates an instance of maverick-api.
    # The actual installation of the maverick-api and dependencies happens in maverick_web::maverick_api

    file { "/srv/maverick/var/log/web/api/${instance}":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    } ->
    file { "/srv/maverick/config/web/api-env.${instance}.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        mode        => "644",
        content     => template("maverick_web/api.conf.erb"),
        notify      => Service["maverick-api@${instance}"],
        replace     => $replaceconfig,
    } ->
    file { "/srv/maverick/config/web/maverick-api.${instance}.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        mode        => "644",
        content     => template("maverick_web/maverick-api.conf.erb"),
        notify      => Service["maverick-api@${instance}"],
        replace     => $replaceconfig,
    } 

    if $active == true {
        service { "maverick-api@${instance}":
            ensure      => running,
            enable      => true,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-api@.service"] ]
        }
    } else {
        service { "maverick-api@${instance}":
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
        require                 => [ Class["maverick_web::maverick_api"], Class["maverick_web::maverick_web_legacy"], Class["nginx"], Service["nginx"] ],
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
