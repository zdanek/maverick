# @summary
#   Maverick_web::Maverick_api class
#   This class installs and manages the Maverick-api software.
#
# @see
#   https://github.com/goodrobots/maverick-api
#
# @example Declaring the class
#   This class is included from maverick_web class and should not be included from elsewhere
#
# @param system_api
#   If true (default), create the default maverick-only system -api instance.
#
class maverick_web::maverick_api (
    $system_api = true,
) {

    # Install python components

    oncevcsrepo { "git-maverick-api":
        gitsource   => "https://github.com/goodrobots/maverick-api.git",
        dest        => "/srv/maverick/software/maverick-api",
        revision    => "master",
        depth       => undef,
    }
    # Install psystemd pip dependency
    ensure_packages(["libsystemd-dev"])

    install_python_module { "api-tornado":
        pkgname     => "tornado",
        ensure      => atleast,
        version     => "6.0.3",
    } ->
    install_python_module { "api-graphql-core":
        pkgname     => "graphql-core",
        ensure      => atleast,
        version     => "3.1.0",
    } ->
    install_python_module { "api-pyjwt":
        pkgname     => "PyJWT",
        ensure      => atleast,
        version     => "1.7.1",
    } ->
    install_python_module { "api-bcrypt":
        pkgname     => "bcrypt",
        ensure      => atleast,
        version     => "3.1.7",
    } ->
    install_python_module { "api-cryptography":
        pkgname     => "cryptography",
        ensure      => atleast,
        version     => "2.9.2",
    } ->
    install_python_module { "api-aiosqlite":
        pkgname     => "aiosqlite",
        ensure      => atleast,
        version     => "0.12.0",
    } ->
    install_python_module { "api-jsoncomment":
        pkgname     => "jsoncomment",
        ensure      => atleast,
        version     => "0.4.2",
    } ->
    install_python_module { "api-jsondiff":
        pkgname     => "jsondiff",
        ensure      => atleast,
        version     => "1.2.0",
    } ->
    install_python_module { "api-psystemd":
        pkgname     => "pystemd",
        ensure      => atleast,
        version     => "0.7.0",
    } ->
    install_python_module { "api-zeroconf":
        pkgname     => "zeroconf",
        ensure      => atleast,
        version     => "0.24.5",
    } ->

    file { "/etc/systemd/system/maverick-api@.service":
        owner       => "root",
        group       => "root",
        mode        => "644",
        source      => "puppet:///modules/maverick_web/maverick-api@.service",
        notify      => Exec["maverick-systemctl-daemon-reload"],
        require     => Oncevcsrepo["git-maverick-api"],
    } -> 
    # Create a symlink to api launch script
    file { "/srv/maverick/software/maverick/bin/api.sh":
        ensure      => link,
        target      => "/srv/maverick/software/maverick/manifests/maverick-modules/maverick_web/files/api.sh",
    } ->

    file { "/srv/maverick/var/log/web/api":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    }

    # Create an -api instance 
    if $system_api == true {
        maverick_web::api { "api-system":
            instance    => "system",
            active      => true,
            apiport     => 6003,
            apiport_ssl => 6004,
            devmode     => false,
            debug       => false,
            replaceconfig    => false,
            config_template  => "maverick-api.system.conf.erb",
        }
        file { "/srv/maverick/software/maverick/bin/status.d/120.web/104.api.status":
            owner   => "mav",
            content => "api@system,MavAPI (System)\n",
        }
    }

}
