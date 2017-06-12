class base::services {
    # Install systemd in Ubuntu 14.04
    if $operatingsystem == "Ubuntu" and ($operatingsystemrelease == "14.04" or $operatingsystemrelease == "14.10") {
        package { ["systemd", "systemd-services"]:
            ensure  => installed
        }
    }

    # Here is where we would disable any services by default
    # Disable cups by default.  It breaks things like iptables and who prints on a UAV?!
    service_stop { ["cups", "cups-browsed"]:
    } ->
    package { ["cups", "cups-filters", "cups-filters-core-drivers", "printer-driver-*", "cups-daemon"]:
        ensure      => purged,
    }
    # Stop postfix, we don't need a mail service/relay for now
    service_stop { "postfix": }

    # Here is where we would enable any services by default
    # service { "example":
    #   ensure      => running,
    #   enable      => true
    # }

}