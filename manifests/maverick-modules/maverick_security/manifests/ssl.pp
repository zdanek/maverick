class maverick_security::ssl (
    $create_ca = true,
    $ca_passphrase = "iceman",
    $ca_country = "US",
    $ca_state = "State of Being",
    $ca_locality = "Moving frequently",
    $ca_orgname = "Maverick CA",
    $ca_orgunit = "Security",
    $ca_cname = "maverick-ca",
) {

    class { "openssl": }

    file { "/srv/maverick/data/security/ssl":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    }

    if $create_ca == true {
        file { "/srv/maverick/data/security/ssl/ca":
            ensure      => directory,
            owner       => "mav",
            group       => "mav",
            mode        => "755",
        }

        # Create Diffie Hellman parameters
        openssl::dhparam { "/srv/maverick/data/security/ssl/dhparam.pem": 
            size        => 2048,
        }
        
        # Create CA.  openssl provider doesn't do CAs, so we have to do it manually
        exec { "create-ca-key":
            command     => "/usr/bin/openssl genrsa -passout pass:${ca_passphrase} -des3 -out /srv/maverick/data/security/ssl/ca/mavCA.key 2048",
            creates     => "/srv/maverick/data/security/ssl/ca/mavCA.key",
        } ->
        file { "/srv/maverick/data/security/ssl/ca/mavCA.key":
            mode        => "600",
        } ->
        exec { "create-ca-rootcert":
            command     => "/usr/bin/openssl req -x509 -new -nodes -passin pass:${ca_passphrase} -key /srv/maverick/data/security/ssl/ca/mavCA.key -sha512 -days 9999 -out /srv/maverick/data/security/ssl/ca/mavCA.pem -subj \"/C=${ca_country}/ST=${ca_state}/L=${ca_locality}/O=${ca_orgname}/OU=${ca_orgunit}/CN=${ca_cname}\"",
            creates     => "/srv/maverick/data/security/ssl/ca/mavCA.pem",
        }
    
    }

}