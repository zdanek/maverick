class base::services {
    
    # Here is where we would disable any services by default
    # Disable cups by default.  It breaks things like iptables and who prints on a UAV?!
    service { ["cups", "cups-browsed"]:
        ensure      => stopped,
        enable      => false
    } ->
    package { ["cups", "cups-filters", "cups-filters-core-drivers", "printer-driver-*", "cups-daemon"]:
        ensure      => purged,
    }
    # Stop postfix, we don't need a mail service/relay for now
    service { "postfix":
        ensure      => stopped,
        enable      => false,
    }


    # Here is where we would enable any services by default
    # service { "example":
    #   ensure      => running,
    #   enable      => true
    # }
        
    # If in flight mode as opposed to dev mode, specifically disable dev mode.  We do it here because
    #  if dev module isn't included it can't disable itself.
    # If you include dev module in flight mode (which is never intended by design), you will probably get a class clash here.
    if $environment == "flight" {
        service { "maverick-cloud9":
            ensure      => stopped,
            enable      => false,
        }
        service { "maverick-dev-sitl":
            ensure      => stopped,
            enable      => false,
        }
        #service { "maverick-mavros-sitl":
        #    ensure      => stopped,
        #    enable      => false,
        #}
    }
    
}