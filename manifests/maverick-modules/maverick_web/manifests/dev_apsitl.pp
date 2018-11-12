class maverick_web::ssl (
    $cert_country = "US",
    $cert_state = "State of Being",
    $cert_locality = "Moving frequently",
    $cert_orgname = "Maverick",
    $cert_orgunit = "Robots",
    $cert_cname = $maverick_web::server_fqdn,
) {

    file { "/srv/maverick/data/web/ssl":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "750",
    }

    # Retrieve CA passphrase for signing
    $ca_passphrase = getvar("maverick_security::ssl::ca_passphrase")

    exec { "create-webssl-key":
        command     => "/usr/bin/openssl genrsa -out /srv/maverick/data/web/ssl/${cert_cname}-webssl.key 2048",
        creates     => "/srv/maverick/data/web/ssl/${cert_cname}-webssl.key",
    } ->
    exec { "create-webssl-csr":
        command     => "/usr/bin/openssl req -new -key /srv/maverick/data/web/ssl/${cert_cname}-webssl.key -out /srv/maverick/data/web/ssl/${cert_cname}-webssl.csr -subj \"/C=${cert_country}/ST=${cert_state}/L=${cert_locality}/O=${cert_orgname}/OU=${cert_orgunit}/CN=${cert_cname}\"",
        creates     => "/srv/maverick/data/web/ssl/${cert_cname}-webssl.csr",
    } ->
    file { "/srv/maverick/data/web/ssl/${cert_cname}-webssl.ext":
        ensure      => present,
        content     => template("maverick_web/webssl.ext.erb"),
    } ->
    exec { "create-webssl-cert":
        command     => "/usr/bin/openssl x509 -req -passin pass:${ca_passphrase} -in /srv/maverick/data/web/ssl/${cert_cname}-webssl.csr -CA /srv/maverick/data/security/ssl/ca/mavCA.pem -CAkey /srv/maverick/data/security/ssl/ca/mavCA.key -CAcreateserial -out /srv/maverick/data/web/ssl/${cert_cname}-webssl.crt -days 365 -sha512 -extfile /srv/maverick/data/web/ssl/${cert_cname}-webssl.ext",
        creates     => "/srv/maverick/data/web/ssl/${cert_cname}-webssl.crt",
        require     => Exec["create-ca-rootcert"],
        notify      => Service["maverick-nginx"]
    }

}
