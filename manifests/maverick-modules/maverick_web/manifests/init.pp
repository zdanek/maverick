class maverick_web (
    $cloud9 = true,
    $nodejs = true,
    $webserver = true,
    $webserver_type = "nginx",
    $webserver_port = 80,
    $webserver_sslport = 443,
) {
    
    if $nodejs == true {
        class { "maverick_web::nodejs": }
    }
    
    if $cloud9 == true {
        class { "maverick_web::cloud9": }
    }
    
    if $webserver == true {
        if $webserver_type == "nginx" {
            class { "maverick_web::nginx": 
                port    => $webserver_port,
                sslport => $webserver_sslport,
            }
        } elsif $webserver_type == "apache" {
            class { "maverick_web::apache":
                port    => $webserver_port,
                sslport => $webserver_sslport,
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
    
}