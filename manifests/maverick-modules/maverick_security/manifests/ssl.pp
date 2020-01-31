# @summary
#   Maverick_security::Ssl class
#   This class installs and manages the Maverick SSL environment.
#
# @example Declaring the class
#   This class is included from maverick_security class and should not be included from elsewhere
#
# @param create_ca
#   If true, create and configure a central Maverick Certificate Authority, which can sign other SSL certificates.
# @param ca_passphrase
#   Passphrase used to protect access to the CA.
# @param ca_country
#   Country field used when creating CA.
# @param ca_state
#   State field used when creating CA.
# @param ca_locality
#   Locality field used when creating CA.
# @param ca_orgname
#   Organisation Name field used when creating CA.
# @param ca_orgunit
#   Organisation Unit field used when creating CA.
# @param ca_cname
#   Canonical Name field used when creating CA.
# 
class maverick_security::ssl (
    Boolean $create_ca = true,
    String $ca_passphrase = "iceman",
    String $ca_country = "US",
    String $ca_state = "State of Being",
    String $ca_locality = "Moving frequently",
    String $ca_orgname = "Maverick CA",
    String $ca_orgunit = "Security",
    String $ca_cname = "MaverickCA ${::hostname}",
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
            fastmode    => true,
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
