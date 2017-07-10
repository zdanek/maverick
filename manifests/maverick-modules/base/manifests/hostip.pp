class base::hostip (
    $additional_entries = {},
) {
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
    if ! empty(lookup("fqdn")) and lookup('fqdn') != $::fqdn {
        $_fqdn = lookup("fqdn")
        $host_entries = {
            "${_fqdn}" => {
                ip              => $_ipaddress,
                host_aliases    => $_host_aliases,
            }
        }
    } else {
        $_fqdn = $::fqdn
        $host_entries = {}
    }
    # Merge in default ipv6 entries
    $_ip6_stdaliases = {
        "ip6-localnet"      => { ip => "fe00::0" },
        "ip6-mcastprefix"   => { ip => "ff00::0" },
        "ip6-allnodes"      => { ip => "ff02::1"},
        "ip6-allrouters"    => { ip => "ff02::2"},
    }
    $_host_entries = merge($host_entries, $_ip6_stdaliases, $additional_entries)
    
    # Only update entry in /etc/hosts if we have all the data necessary
    if $_fqdn and $_ipaddress and $_hostname { 
        class { "::hosts":
            use_fqdn            => true,
            purge_hosts         => true,
            stored_config       => false,
            localhost_aliases   => ["localhost"],
            localhost6_aliases  => ["localhost", "ip6-localhost", "ip6-loopback"],
            fqdn_host_aliases   => $_host_aliases,
            host_entries        => $_host_entries,
        }
    }

}
