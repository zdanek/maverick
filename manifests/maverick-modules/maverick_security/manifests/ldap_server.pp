class maverick_security::ldap_server (
    $server_type = "openldap",
    $base = "dc=maverick,dc=one",
    # Create sha hashes here: http://www.mytecbits.com/tools/cryptography/sha2generator
    $roothash = "{SHA512}8dba3660197daf7642b9eaeaf77ef817ae9471f3b3d672bfb4fd94100dd5801cd9c990dd64938822e17643b8810cd31ee0ae87b9370884a7d880cfbc5307a573", # youcanbemywingman
    $cert_country = "US",
    $cert_state = "State of Being",
    $cert_locality = "Moving frequently",
    $cert_orgname = "Maverick",
    $cert_orgunit = "Robots",
    $cert_cname = "slapd",
    $server_fqdn = $::fqdn,
) {
    
    # Create ldap ssl and data directory
    file { ["/srv/maverick/data/security/ldap", "/srv/maverick/data/security/ssl/ldap"]:
    	ensure		=> directory,
    	owner		=> "mav",
    	group		=> "mav",
    	mode		=> "755",
    } ->
    # Symlink the default ldap config directory for clients
    file { "/etc/ldap/slapd.d":
        ensure      => symlink,
        target      => "/srv/maverick/config/security/ldap",
        force       => true,
    }
    
    # Retrieve CA passphrase for signing
    $ca_passphrase = getvar("maverick_security::ssl::ca_passphrase")

    # Create cert chain
    exec { "create-ldapssl-key":
        command     => "/usr/bin/openssl genrsa -out /srv/maverick/data/security/ssl/ldap/${cert_cname}-ldapssl.key 2048",
        creates     => "/srv/maverick/data/security/ssl/ldap/${cert_cname}-ldapssl.key",
        user        => "mav",
    } ->
    exec { "create-ldapssl-csr":
        command     => "/usr/bin/openssl req -new -key /srv/maverick/data/security/ssl/ldap/${cert_cname}-ldapssl.key -out /srv/maverick/data/security/ssl/ldap/${cert_cname}-ldapssl.csr -subj \"/C=${cert_country}/ST=${cert_state}/L=${cert_locality}/O=${cert_orgname}/OU=${cert_orgunit}/CN=${cert_cname}\"",
        creates     => "/srv/maverick/data/security/ssl/ldap/${cert_cname}-ldapssl.csr",
        user        => "mav",
    } ->
    file { "/srv/maverick/data/security/ssl/ldap/${cert_cname}-ldapssl.ext":
        ensure      => present,
        content     => template("maverick_security/ldapssl.ext.erb"),
        owner       => "mav",
        group       => "mav",
    } ->
    exec { "create-ldapssl-cert":
        command     => "/usr/bin/openssl x509 -req -passin pass:${ca_passphrase} -in /srv/maverick/data/security/ssl/ldap/${cert_cname}-ldapssl.csr -CA /srv/maverick/data/security/ssl/ca/mavCA.pem -CAkey /srv/maverick/data/security/ssl/ca/mavCA.key -CAcreateserial -out /srv/maverick/data/security/ssl/ldap/${cert_cname}-ldapssl.crt -days 365 -sha512 -extfile /srv/maverick/data/security/ssl/ldap/${cert_cname}-ldapssl.ext",
        creates     => "/srv/maverick/data/security/ssl/ldap/${cert_cname}-ldapssl.crt",
        require     => Exec["create-ca-rootcert"],
        user        => "mav",
        notify      => Service["slapd"]
    }

    # Dependency for augeas shellvar provider
    ensure_packages(["ruby-augeas"])
    
    if $server_type == "openldap" {
        # Setup rsyslog logging for openldap slapd
        file { "/etc/rsyslog.d/10-slapd.conf":
            content  => "local4.* /var/log/slapd.log\n",
            #notify   => Service["rsyslog"],
        } ->
        file { "/var/log/slapd.log":
            ensure   => present,
            owner    => root,
            group    => root,
            mode     => "0644",
        } ->
        file { "/etc/default/slapd":
            source   => "puppet:///modules/maverick_security/default.slapd",
        } ->
        class { 'openldap::server': 
            confdir   => '/srv/maverick/config/security/ldap',
            provider  => 'olc',
            owner     => 'mav',
            group     => 'mav',
            ldaps_ifs => ['/'],
            ssl_ca    => '/srv/maverick/data/security/ssl/ca/mavCA.pem',
            ssl_cert  => '/srv/maverick/data/security/ssl/ldap/slapd-ldapssl.crt',
            ssl_key   => '/srv/maverick/data/security/ssl/ldap/slapd-ldapssl.key',
            require   => Package["ruby-augeas"],
        }
        openldap::server::database { $base:
          ensure    => present,
          rootdn    => "cn=admin,${base}",
          rootpw    => $roothash,
          directory => "/srv/maverick/data/security/ldap/maverick.one",
        }
        openldap::server::globalconf { 'LogLevel':
          ensure => present,
          value  => '1535',
        }
        /*
        openldap::server::globalconf { 'Security':
            ensure  => present,
        	value   => { 'Security' => [ 'simple_bind=128', 'ssf=128', 'tls=0' ] } 
        }
        */
        openldap::server::schema { 'inetorgperson':
          ensure  => present,
          path    => '/etc/ldap/schema/inetorgperson.schema',
        }
        openldap::server::access { "0 on ${base}":
          what     => 'attrs=userPassword,shadowLastChange',
          access   => [
            "by dn=\"cn=admin,${base}\" write",
            'by anonymous auth',
            'by self write',
            'by * none',
          ],
        }
    }
    
}