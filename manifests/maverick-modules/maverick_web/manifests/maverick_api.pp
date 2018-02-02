class maverick_web::maverick_api (
    $active = true,
    $apiport = 6795,
) {

    # install python components
    install_python_module { "mavapi-graphene":
        pkgname     => "graphene",
        ensure      => atleast,
        version     => "2.0",
    } ->
    install_python_module { "mavapi-sqlalchemy":
        pkgname     => "SQLAlchemy",
        ensure      => present,
    } ->
    install_python_module { "mavapi-graphene-sqlalchemy":
        pkgname     => "graphene-sqlalchemy",
        ensure      => present,
    } ->
    install_python_module { "mavapi-tornado":
        pkgname     => "tornado",
        ensure      => present,
    } ->
    install_python_module { "mavapi-rx":
        pkgname     => "rx",
        ensure      => present,
    } ->
    install_python_module { "mavapi-zeroconf":
        pkgname     => "zeroconf",
        ensure      => present,
    } ->
    oncevcsrepo { "git-maverick-api":
        gitsource   => "https://github.com/goodrobots/maverick-api.git",
        dest        => "/srv/maverick/code/maverick-api",
        revision    => "master",
        depth       => undef,
    } ->
    file { "/etc/systemd/system/maverick-api.service":
        owner       => "root",
        group       => "root",
        mode        => "644",
        source      => "puppet:///modules/maverick_web/maverick-api.service",
        notify      => Exec["maverick-systemctl-daemon-reload"],
    } -> 
    nginx::resource::location { "maverick-api":
        location    => "/maverick-api/",
        proxy       => "http://localhost:${apiport}/",
        server      => "${::hostname}.local",
        require     => [ Class["maverick_gcs::fcs"], Class["nginx"] ],
    }
    
    if $active == true {
        service { "maverick-api":
            ensure      => running,
            enable      => true,
            require     => Exec["maverick-systemctl-daemon-reload"],
        }
    } else {
        service { "maverick-api":
            ensure      => stopped,
            enable      => false,
            require     => Exec["maverick-systemctl-daemon-reload"],
        }
    }
    
    if defined(Class["::maverick_security"]) {
        maverick_security::firewall::firerule { "maverick-api":
            ports       => $apiport,
            ips         => lookup("firewall_ips"),
            proto       => "tcp"
        }
    }

}