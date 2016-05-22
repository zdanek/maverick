class base::functions {

    ## Define an apt-get update that can be notified, but is only run if notified
    exec { "maverick-aptget-update":
        command         => "/usr/bin/apt-get update",
        refreshonly     => true,
    }
    
    # Define an exec to do systemctl daemon-reload that can be called through notify
    # Make sure to name it something that won't clash elsewhere, and don't exec unless notified
    exec { "maverick-systemctl-daemon-reload":
        command         => "/bin/systemctl daemon-reload",
        refreshonly     => true,
    }

}