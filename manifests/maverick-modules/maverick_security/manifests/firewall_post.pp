# @summary
#   Maverick_security::firewall_post class
#   This class declares post-firewall rules
#
# @example Declaring the class
#   This class is included from maverick_security::firewall class and should not be included from elsewhere
#
class maverick_security::firewall_post {
    
    firewall { '999 drop all':
        proto   => 'all',
        action  => 'drop',
        before  => undef,
    }
  
}
