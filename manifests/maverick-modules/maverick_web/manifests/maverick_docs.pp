class maverick_web::maverick_docs (
    $server_hostname = $maverick_web::server_fqdn,
) {

    file { "/srv/maverick/software/maverick-docs":
        ensure      => absent,
        force       => true,
    }

    nginx::resource::location { "web-maverick-docs":
        ensure          => present,
        ssl             => true,
        location        => "/web/maverick-docs",
        location_alias  => "/srv/maverick/software/maverick/docs",
        index_files     => ["index.html"],
        server          => $server_hostname,
        require         => [ Class["maverick_gcs::fcs"], Class["nginx"] ],
    }

}