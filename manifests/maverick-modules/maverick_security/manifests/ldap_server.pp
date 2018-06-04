class maverick_security::ldap_server (
    $server_type = "openldap",
    $base = "dc=maverick,dc=one",
    # Create sha hashes here: http://www.mytecbits.com/tools/cryptography/sha2generator
    $roothash = "{SHA512}8dba3660197daf7642b9eaeaf77ef817ae9471f3b3d672bfb4fd94100dd5801cd9c990dd64938822e17643b8810cd31ee0ae87b9370884a7d880cfbc5307a573", # youcanbemywingman
) {
    
    # Create ldap config and data directory
    file { ["/srv/maverick/data/security/ldap"]:
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

    # Dependency for augeas shellvar provider
    ensure_packages(["ruby-augeas"])
    
    if $server_type == "openldap" {
        file { "/etc/default/slapd":
            source   => "puppet:///modules/maverick_security/default.slapd",
        } ->
        class { 'openldap::server': 
            confdir  => '/srv/maverick/config/security/ldap',
            provider => 'olc',
            owner    => 'mav',
            group    => 'mav',
            require  => Package["ruby-augeas"],
        }
        openldap::server::database { $base:
          ensure    => present,
          rootdn    => "cn=admin,${base}",
          rootpw    => $roothash,
          directory => "/srv/maverick/data/security/ldap/maverick.one",
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