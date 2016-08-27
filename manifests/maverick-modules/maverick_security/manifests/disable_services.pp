class maverick_security::disable_services {
    service { ["atd", "autofs", "haldaemon", "avahi-daemon", "dnsmasq", "portreserve", "named"]:
        ensure => stopped,
        enable => false,
    }

}