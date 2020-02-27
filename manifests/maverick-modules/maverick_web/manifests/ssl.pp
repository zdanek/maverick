# @summary
#   Maverick_web::Ssl class
#   This class installs and manages the web SSL configuration.
#
# @example Declaring the class
#   This class is included from maverick_web class and should not be included from elsewhere
#
class maverick_web::ssl (
    $cert_country = "US",
    $cert_state = "State of Being",
    $cert_locality = "Moving frequently",
    $cert_orgname = "Maverick",
    $cert_orgunit = "Robots",
    $cert_cname = $maverick_web::server_fqdn,
) {
    $ssl_location = getvar("maverick_web::ssl_location")

    file { $ssl_location:
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "750",
    }

    # Retrieve CA passphrase for signing
    $ca_passphrase = getvar("maverick_security::ssl::ca_passphrase")

    exec { "create-webssl-key":
        command     => "/usr/bin/openssl genrsa -out ${ssl_location}/${cert_cname}-webssl.key 2048",
        creates     => "${ssl_location}/${cert_cname}-webssl.key",
        user        => "mav",
    } ->
    file { "${ssl_location}/mavweb.key":
        ensure      => symlink,
        target      => "${ssl_location}/${cert_cname}-webssl.key",
        owner       => "mav",
        group       => "mav",
    } ->
    exec { "create-webssl-csr":
        command     => "/usr/bin/openssl req -new -key ${ssl_location}/${cert_cname}-webssl.key -out ${ssl_location}/${cert_cname}-webssl.csr -subj \"/C=${cert_country}/ST=${cert_state}/L=${cert_locality}/O=${cert_orgname}/OU=${cert_orgunit}/CN=${cert_cname}\"",
        creates     => "${ssl_location}/${cert_cname}-webssl.csr",
        user        => "mav",
    } ->
    file { "${ssl_location}/${cert_cname}-webssl.ext":
        ensure      => present,
        content     => template("maverick_web/webssl.ext.erb"),
        owner       => "mav",
        group       => "mav",
    } ->
    exec { "create-webssl-cert":
        command     => "/usr/bin/openssl x509 -req -passin pass:${ca_passphrase} -in ${ssl_location}/${cert_cname}-webssl.csr -CA /srv/maverick/data/security/ssl/ca/mavCA.pem -CAkey /srv/maverick/data/security/ssl/ca/mavCA.key -CAcreateserial -out ${ssl_location}/${cert_cname}-webssl.crt -days 365 -sha512 -extfile ${ssl_location}/${cert_cname}-webssl.ext",
        creates     => "${ssl_location}/${cert_cname}-webssl.crt",
        require     => Exec["create-ca-rootcert"],
        notify      => Service["maverick-nginx"],
    } ->
    file { "${ssl_location}/mavweb.crt":
        ensure      => symlink,
        target      => "${ssl_location}/${cert_cname}-webssl.crt",
        owner       => "mav",
        group       => "mav",
    } ->
    # Reset all web ssl files to mav user - they are root because we need access to the CA cert to sign which is owned by root
    exec { "web-ssl-ownerships":
        command     => "/bin/chown -R mav:mav $ssl_location",
        onlyif      => "/bin/ls -l $ssl_location |grep root",
    }

}
