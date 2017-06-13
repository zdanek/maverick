class maverick_security::disable_services {
    service_wrapper { ["atd", "autofs", "haldaemon", "avahi-daemon", "dnsmasq", "portreserve", "named"]:
        ensure => stopped,
        enable => false,
    }

}