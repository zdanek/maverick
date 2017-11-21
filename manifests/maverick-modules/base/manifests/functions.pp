class base::functions {

    # Define an exec to do systemctl daemon-reload that can be called through notify
    # Make sure to name it something that won't clash elsewhere, and don't exec unless notified
    exec { "maverick-systemctl-daemon-reload":
        command         => "/bin/systemctl daemon-reload",
        refreshonly     => true,
    }
    
    # Define an exec to do systemctl reset-failed that can be called through notify
    exec { "maverick-systemctl-reset-failed":
        command         => "/bin/systemctl reset-failed",
        refreshonly     => true,
    }
    
    # Define an exec to do ldconfig, to reload ld.so.conf.d config
    exec { "maverick-ldconfig":
        command         => "/sbin/ldconfig",
        refreshonly     => true,
    }

}