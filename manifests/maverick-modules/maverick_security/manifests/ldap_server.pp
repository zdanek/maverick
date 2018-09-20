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
    
    # Retrieve CA passphrase for signing
    $ca_passphrase = getvar("maverick_security::ssl::ca_passphrase")

    # Create ldap ssl and data directory
    file { ["/etc/ldap/ssl"]:
    	ensure		=> directory,
    	owner		=> "openldap",
    	group		=> "openldap",
    	mode		=> "755",
    } ->

    # Create cert chain
    exec { "create-ldapssl-key":
        command     => "/usr/bin/openssl genrsa -out /etc/ldap/ssl/${cert_cname}-ldapssl.key 2048",
        creates     => "/etc/ldap/ssl/${cert_cname}-ldapssl.key",
        user        => "openldap",
    } ->
    exec { "create-ldapssl-csr":
        command     => "/usr/bin/openssl req -new -key /etc/ldap/ssl/${cert_cname}-ldapssl.key -out /etc/ldap/ssl/${cert_cname}-ldapssl.csr -subj \"/C=${cert_country}/ST=${cert_state}/L=${cert_locality}/O=${cert_orgname}/OU=${cert_orgunit}/CN=${cert_cname}\"",
        creates     => "/etc/ldap/ssl/${cert_cname}-ldapssl.csr",
        user        => "openldap",
    } ->
    file { "/etc/ldap/ssl/${cert_cname}-ldapssl.ext":
        ensure      => present,
        content     => template("maverick_security/ldapssl.ext.erb"),
        owner       => "openldap",
        group       => "openldap",
    } ->
    exec { "create-ldapssl-cert":
        command     => "/usr/bin/openssl x509 -req -passin pass:${ca_passphrase} -in /etc/ldap/ssl/${cert_cname}-ldapssl.csr -CA /srv/maverick/data/security/ssl/ca/mavCA.pem -CAkey /srv/maverick/data/security/ssl/ca/mavCA.key -CAcreateserial -out /etc/ldap/ssl/${cert_cname}-ldapssl.crt -days 365 -sha512 -extfile /etc/ldap/ssl/${cert_cname}-ldapssl.ext",
        unless      => "/usr/bin/find /etc/ldap/ssl -name ${cert_cname}-ldapssl.crt -size +0 | /bin/egrep '.*'",
        require     => Exec["create-ca-rootcert"],
        notify      => Service["slapd"]
    } ->

    # Ensure resulting cert ownerships are correct
    /*
    exec { "chown-ldapssl":
        command     => "/bin/chown -R openldap /etc/ldap/ssl/*",
        onlyif      => "/bin/ls -l /etc/ldap/ssl/* |awk {'print $3'} |grep -v openldap",
    }
    */
    file { [ "/etc/ldap/ssl/${cert_cname}-ldapssl.key", "/etc/ldap/ssl/${cert_cname}-ldapssl.csr", "/etc/ldap/ssl/${cert_cname}-ldapssl.crt" ]:
        owner       => "openldap",
        group       => "openldap",
        mode        => "0644",
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
        /*
        file { "/etc/default/slapd":
            source   => "puppet:///modules/maverick_security/default.slapd",
        } ->
        */
        file { "/etc/apparmor.d/local/usr.sbin.slapd":
            source   => "puppet:///modules/maverick_security/apparmor.local.usr.sbin.slapd",
            mode     => "0644",
            owner    => "root",
            group    => "root",
        } ->
        class { 'openldap::server': 
            # confdir   => '/srv/maverick/config/security/ldap',
            provider  => 'olc',
            #owner     => 'mav',
            #group     => 'mav',
            ldaps_ifs => ['/'],
            ssl_ca    => '/srv/maverick/data/security/ssl/ca/mavCA.pem',
            ssl_cert  => '/etc/ldap/ssl/slapd-ldapssl.crt',
            ssl_key   => '/etc/ldap/ssl/slapd-ldapssl.key',
            require   => Package["ruby-augeas"],
        }
        openldap::server::database { $base:
          ensure    => present,
          rootdn    => "cn=admin,${base}",
          rootpw    => $roothash,
          # directory => "/srv/maverick/data/security/ldap/maverick.one",
        }
        /*
        openldap::server::globalconf { 'LogLevel':
          ensure => present,
          value  => '1535',
        }
        */
        /*
        openldap::server::globalconf { 'Security':
            ensure  => present,
        	#value   => { 'Security' => [ 'simple_bind=128', 'ssf=128', 'tls=128' ] }
        	#value   => { 'Security' => [ 'simple_bind=128', 'ssf=128' ] } 
        }
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
        */
    }
    
}