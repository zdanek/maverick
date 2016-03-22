class base::hostip {
    # If we have debian loopaddress set, make sure it's set to current hostname
    exec { "loophost":
        unless      => "/bin/grep '127.0.1.1\\s+${hostname}'",
        command     => "/bin/sed /etc/hosts -i -r -e 's/127.0.1.1\\s+(.*)/127.0.1.1\\t${hostname}/'"
    }

    # Retrieve host/ip values from hiera
    $_fqdn = hiera('fqdn')
    $_primaryip = hiera("primaryip")
    $_hostname = hiera("hostname")
    $_hierahostaliases = hiera("hostaliases")

    if $_hierahostaliases != [] {
        $_host_aliases = [$_hostname, $_hierahostaliases]
    } else {
        $_host_aliases = [$_hostname]
    }
    
    # Only update entry in /etc/hosts if we have all the data necessary
    if $_fqdn and $_primaryip and $_hostname { 
        host { "$_fqdn":
            ip              => $_primaryip,
            host_aliases    => $_host_aliases,
        }

    if ($operatingsystem == "CentOS") or ($operatingsystem == "RedHat") or ($operatingsystem == "Fedora") {        
            augeas {"network-hostname":
                 changes => [
                     "set /files/etc/sysconfig/network/HOSTNAME $_fqdn",
               ],
            } 
    }
    
    }

}
