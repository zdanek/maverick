# @summary
#   Maverick_web::Maverick_web class
#   This class installs and manages the Maverick-web software.
#
# @see
#   https://github.com/goodrobots/maverick-web
#
# @example Declaring the class
#   This class is included from maverick_web class and should not be included from elsewhere
#
# @param active
#   If true, start service and enable at boot time.
# @param webport
#   The port to listen for connections.
# @param webpath_dev
#   The web path to use for the dev -web content.
# @param webpath_prod
#   The web path to use for the production -web content.
# @param server_hostname
#   Specify the webserver vhost to use.
# @param auth_message
#   If set, specify the message to use in web auth popup.
# @param auth_file
#   If set, specify the webserver auth file to use to authenticate incoming users.
#
class maverick_web::maverick_web (
    Boolean $active = false,
    Integer $webport = 6100,
    String $webpath_dev = '/dev/maverick',
    String $webpath_prod = '/web/maverick',
    String $server_hostname = $maverick_web::server_fqdn,
    Optional[String] $auth_message = undef,
    Optional[String] $auth_file = undef,
) {
    
    # Install dev repo, register maverick-webdev service
    package { 'yarn':
        ensure   => latest,
        provider => 'npm',
    } ->
    package { '@vue/cli':
        ensure   => 'latest',
        provider => 'npm',
    } ->
    oncevcsrepo { "git-maverick-web":
        gitsource   => "https://github.com/goodrobots/maverick-web.git",
        dest        => "/srv/maverick/code/maverick-web",
        revision    => "master",
        depth       => undef,
    } ->
    exec { "yarn-maverick-web":
        path        => ["/bin", "/usr/bin", "/opt/nodejs/bin"],
        command     => "yarn install",
        cwd         => "/srv/maverick/code/maverick-web",
        creates     => "/srv/maverick/code/maverick-web/node_modules/@vue",
        user        => "mav",
        #environment => ["QT_QPA_PLATFORM=offscreen"], # Fix to allow global phantomjs to run headless
        timeout     => 0,
        require     => [ Class["maverick_web::nodejs"], Package["yarn"], ],
    } ->
    file { "/etc/systemd/system/maverick-webdev.service":
        owner       => "root",
        group       => "root",
        mode        => "644",
        source      => "puppet:///modules/maverick_web/maverick-webdev.service",
        notify      => Exec["maverick-systemctl-daemon-reload"],
    } -> 
    # Define nginx location for webdev proxy websocket endpoint
    nginx::resource::location { "maverick-webdev-ws":
        ssl                     => true,
        location                => "${webpath_dev}/sockjs-node",
        proxy                   => "http://localhost:${webport}/dev/maverick/sockjs-node",
        server                  => $server_hostname,
    	proxy_connect_timeout   => "7d",
    	proxy_read_timeout      => "7d",
        proxy_set_header        => ['Upgrade $http_upgrade', 'Connection "upgrade"', 'Host $host', 'X-Real-IP $remote_addr', 'X-Forwarded-For $proxy_add_x_forwarded_for', 'Proxy ""'],
    	proxy_http_version      => "1.1",
        require                 => [ Class["nginx"], Service["nginx"] ],
    } ->
    # Define nginx location for webdev proxy
    nginx::resource::location { "maverick-webdev":
        location    => $webpath_dev,
        ensure      => present,
        ssl         => true,
        proxy       => "http://localhost:${webport}/dev/maverick/",
        server      => $server_hostname,
        auth_basic  => $auth_message,
        auth_basic_user_file => $auth_file,
        require     => [ Class["nginx"], Service["nginx"] ],
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
        require         => [ Class["nginx"], Service["nginx"] ],
    }

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

    # status.d entry
    file { "/srv/maverick/software/maverick/bin/status.d/120.web/100.webdev.status":
        owner   => "mav",
        content => "webdev,Web Devserver\n",
    }
}
