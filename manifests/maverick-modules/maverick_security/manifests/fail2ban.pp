# @summary
#   Maverick_security::fail2ban class
#   This class installs and manages the fail2ban service.  This provides protection against brute force attacks.
#
# @example Declaring the class
#   This class is included from maverick_security class and should not be included from elsewhere
#
# @param $alertemail
#   Email address to send alerts to.
#
class maverick_security::fail2ban (
        Optional[String] $alertemail = undef,
    ) {

    class { "::fail2ban":
        ignoreip            => lookup("firewall_ips"),
        destemail           => $alertemail,
        bantime             => 1800,
        maxretry            => 6,
    }
    class { "::fail2ban::jail::ssh": }

}
