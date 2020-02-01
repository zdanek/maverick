# @summary
#   Maverick_security::disable_services class
#   This class disables various system services that are typically not needed or a good idea for security.
#
# @example Declaring the class
#   This class is included from maverick_security class and should not be included from elsewhere
#
class maverick_security::disable_services {
    service { [
        "atd",
        "autofs",
        "haldaemon",
        "dnsmasq",
        "portreserve",
        "named",
    ]:
        ensure => stopped,
        enable => false,
    }

    # These are services that run on the raspberry (maybe others as well)
    service { [
        "alsa-state",
        "exim4",
        "thd",
    ]:
        ensure => stopped,
        enable => false,
    }

}
