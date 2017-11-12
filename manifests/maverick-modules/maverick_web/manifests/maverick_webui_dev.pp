class maverick_web::maverick_webui_dev (
    $frontend_dev_port = 6795,
) {
    
    package { 'vue-cli':
        ensure   => 'present',
        provider => 'npm',
    }
    
    file { "/srv/maverick/code/maverick-webui-dev":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    }
    
    if defined(Class["::maverick_security"]) {
        maverick_security::firewall::firerule { "webui-frontend-dev":
            ports       => $frontend_dev_port,
            ips         => lookup("firewall_ips"),
            proto       => "tcp"
        }
    }

}