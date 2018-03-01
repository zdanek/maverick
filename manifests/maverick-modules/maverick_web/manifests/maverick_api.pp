class maverick_web::maverick_api (
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
    file { "/etc/systemd/system/maverick-api@.service":
        owner       => "root",
        group       => "root",
        mode        => "644",
        source      => "puppet:///modules/maverick_web/maverick-api@.service",
        notify      => Exec["maverick-systemctl-daemon-reload"],
    } -> 
    # Create a symlink to api launch script
    file { "/srv/maverick/software/maverick/bin/api.sh":
        ensure      => link,
        target      => "/srv/maverick/software/maverick/manifests/maverick-modules/maverick_web/files/api.sh",
    }
    
    file { "/srv/maverick/var/log/web/api":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    }

}