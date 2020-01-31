# @summary
#   Maverick_security::ldap_client class
#   This class configures an ldap client
#
# @example Declaring the class
#   This class is included from maverick_security class and should not be included from elsewhere
#
# @param base
#   Set the LDAP base DN
#
class maverick_security::ldap_client (
    String $base="dc=maverick,dc=one",
) {
    
    class { 'openldap::client':
      base       => $base,
      uri        => ['ldap://localhost', 'ldapi://localhost'],
      tls_cacert => '/srv/maverick/data/security/ssl/ca/mavCA.pem',
    }
    
}
