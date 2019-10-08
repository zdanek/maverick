class maverick_web (
    $cloud9 = true,
    $codeserver = true,
    $nodejs = true,
    $webserver = true,
    $webserver_type = "nginx",
    $webserver_port = 80,
    $webserver_sslport = 443,
    $maverick_docs = true,
    $ssl = true,
    $maverick_web = true,
    $maverick_api = true,
    $server_fqdn = $::fqdn,
) {
    
    # Create status.d directory for maverick status`
    file { "/srv/maverick/software/maverick/bin/status.d/120.web":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    } ->
    file { "/srv/maverick/software/maverick/bin/status.d/120.web/__init__":
        owner       => "mav",
        content     => "Web Services",
    }

    # Install tornado here as it is used across maverick modules
    install_python_module { "tornado":
        pkgname     => "tornado",
        ensure      => atleast,
        version     => "6.0.3",
    }

    if $nodejs == true {
        class { "maverick_web::nodejs": }
    }
    
    if $cloud9 == true {
        class { "maverick_web::cloud9": }
    }
    
    if $codeserver == true {
        class { "maverick_web::codeserver": }
    }

    file { [ "/srv/maverick/data/web", "/srv/maverick/config/web", "/srv/maverick/var/log/web", "/srv/maverick/var/lib/web", ]:
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    }
    
    if $webserver == true {
        if $webserver_type == "nginx" {
            class { "maverick_web::nginx": 
                port    => $webserver_port,
                ssl_port => $webserver_sslport,
            }
        } elsif $webserver_type == "apache" {
            class { "maverick_web::apache":
                port    => $webserver_port,
                ssl_port => $webserver_sslport,
            }
        }
        
        # Create hole in firewall for webserver
        if defined(Class["::maverick_security"]) {
            maverick_security::firewall::firerule { "webserver":
                ports       => [$webserver_port, $webserver_sslport],
                ips         => lookup("firewall_ips"),
                proto       => "tcp"
            }
        }

    }
    
    if $maverick_docs == true {
        class { "maverick_web::maverick_docs": }
    }
    
    if $ssl == true {
        class { "maverick_web::ssl": }
    }
    
    if $maverick_api == true {
        class { "maverick_web::maverick_api": }
    }

    if $maverick_web == true {
        class { "maverick_web::maverick_web": }
    }
    
}
