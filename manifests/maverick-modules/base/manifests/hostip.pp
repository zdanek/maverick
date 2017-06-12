class base::hostip {
    # If we have debian 127.0.1.1 loopaddress set, make sure it's set to current hostname
    exec { "loophost11":
        onlyif      => "/bin/grep '127.0.1.1' /etc/hosts |/bin/grep -vE '127.0.1.1\\s+${hostname}'",
        command     => "/bin/sed /etc/hosts -i -r -e 's/127.0.1.1\\s+(.*)/127.0.1.1\\t${hostname}/'"
    }

    # If we have debian 127.0.0.1 loopaddress set, make sure it's set to current hostname but ignore localhost
    exec { "loophost01":
        onlyif      => "/bin/grep '127.0.0.1' /etc/hosts |/bin/grep -v localhost |/bin/grep -vE '127.0.0.1\\s+${hostname}'",
        command     => "/bin/sed /etc/hosts -i -r -e '/localhost/! s/127.0.0.1\\s+(.*)/127.0.0.1\\t${hostname}/'"
    }

    # Retrieve host/ip values from hiera
    if ! empty(lookup("fqdn")) {
        $_fqdn = lookup("fqdn")
    } else {
        $_fqdn = $::fqdn
    }
    if ! empty(lookup("primaryip")) {
        $_ipaddress = lookup("primaryip")
    } else {
        $_ipaddress = $::ipaddress
    }
    if ! empty(lookup("hostname")) {
        $_hostname = lookup("hostname")
    } else {
        $_hostname = $::hostname
    }
    $_hierahostaliases = lookup("hostaliases")

    if ! empty($_hierahostaliases) {
        $_host_aliases = [$_hostname, $_hierahostaliases]
    } else {
        $_host_aliases = [$_hostname]
    }
    
    # Only update entry in /etc/hosts if we have all the data necessary
    if $_fqdn and $_ipaddress and $_hostname { 
        host { $_fqdn:
            ip              => $_ipaddress,
            host_aliases    => $_host_aliases,
        }
    }

}
