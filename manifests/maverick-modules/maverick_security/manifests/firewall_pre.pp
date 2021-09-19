# @summary
#   Maverick_security::firewall_pre class
#   This class declares pre-firewall rules
#
# @example Declaring the class
#   This class is included from maverick_security::firewall class and should not be included from elsewhere
#
class maverick_security::firewall_pre {

    Firewall {
        require => undef,
    }

    # Default firewall rules
    firewall { '000 accept all icmp':
        proto   => 'icmp',
        action  => 'accept',
    } ->
    firewall { '001 accept all to lo interface':
        proto   => 'all',
        iniface => 'lo',
        action  => 'accept',
    } ->
    firewall { '002 reject local traffic not on loopback interface':
        iniface     => '! lo',
        proto       => 'all',
        destination => '127.0.0.1/8',
        action      => 'reject',
    } ->
    firewall { '003 accept related established rules':
        proto   => 'all',
        state   => ['RELATED', 'ESTABLISHED'],
        action  => 'accept',
    }

}
