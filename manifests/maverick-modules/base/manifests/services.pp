class base::services (
         
    ) {
    
    # Here is where we would disable any services by default
    # service { "example":
    #   ensure      => stopped,
    #   enable      => false
    # }
    # Disable cups by default.  It breaks things like iptables and who prints on a UAV?!
    service { ["cups", "cups-browsed"]:
        ensure      => stopped,
        enable      => false
    } ->
    package { ["cups", "cups-filters", "cups-filters-core-drivers", "printer-driver-*", "cups-daemon"]:
        ensure      => purged,
    }

    # Here is where we would enable any services by default
    # service { "example":
    #   ensure      => running,
    #   enable      => true
    # }
        
    # Define an exec to do systemctl daemon-reload that can be called through notify
    # Make sure to name it something that won't clash elsewhere, and don't exec unless notified
    exec { "maverick-systemctl-daemon-reload":
        command         => "/bin/systemctl daemon-reload",
        refreshonly     => true,
    }
}