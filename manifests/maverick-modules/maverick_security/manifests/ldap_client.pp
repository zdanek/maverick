class maverick_security::ldap_client (
    $base="dc=maverick,dc=one",
) {
    
    class { 'openldap::client':
      base       => $base,
      uri        => ['ldap://localhost', 'ldapi://localhost'],
      tls_cacert => '/srv/maverick/data/security/ssl/ca/mavCA.pem',
    }
    
}