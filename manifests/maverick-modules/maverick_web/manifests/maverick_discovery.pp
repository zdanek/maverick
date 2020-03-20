# @summary
#   Maverick_web::Maverick_discovery class
#   This class installs and manages the Maverick-discovery software.
#
# @see
#   https://github.com/goodrobots/maverick-discovery
#
# @example Declaring the class
#   This class is included from maverick_web class and should not be included from elsewhere
#
# @param active
#   If true, start service and enable at boot time.
#
class maverick_web::maverick_discovery (
    Boolean $active = true,
) {

    # Install python components
    oncevcsrepo { "git-maverick-discovery":
        gitsource   => "https://github.com/goodrobots/maverick-discovery.git",
        dest        => "/srv/maverick/software/maverick-discovery",
        revision    => "master",
    } ->

    install_python_module { "discovery-tornado":
        pkgname     => "tornado",
        ensure      => atleast,
        version     => "6.0.4",
    } ->
    install_python_module { "discovery-zeroconf":
        pkgname     => "zeroconf",
        ensure      => atleast,
        version     => "0.24.5",
    } ->

    file { "/etc/systemd/system/maverick-discovery.service":
        owner       => "root",
        group       => "root",
        mode        => "644",
        source      => "puppet:///modules/maverick_web/maverick-discovery.service",
        notify      => Exec["maverick-systemctl-daemon-reload"],
        require     => Oncevcsrepo["git-maverick-discovery"],
    }

    # Bring discovery service to desired state
    if $active == true {
        service { "maverick-discovery":
            ensure      => running,
            enable      => true,
            require     => Exec["maverick-systemctl-daemon-reload"],
        }
    } else {
        service { "maverick-discovery":
            ensure      => stopped,
            enable      => false,
            require     => Exec["maverick-systemctl-daemon-reload"],
        }
    }

    # status.d entry
    file { "/srv/maverick/software/maverick/bin/status.d/120.web/110.discovery.status":
        owner   => "mav",
        content => "discovery,Web Discovery Service\n",
    }
}
