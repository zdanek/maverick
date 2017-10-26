class maverick_web::maverick_docs (
) {
    
    # Install maverick docs
    oncevcsrepo { "github-maverick_docs":
        gitsource   => "https://github.com/fnoop/maverick",
        dest        => "/srv/maverick/software/maverick-docs",
        revision    => "gh-pages",
    }

    nginx::resource::location { "web-maverick-docs":
        ensure          => present,
        ssl             => true,
        location        => "/web/maverick-docs",
        location_alias  => "/srv/maverick/software/maverick-docs",
        index_files     => ["maverick.html"], # index.html is for github hosted docs
        server          => "${::hostname}.local",
        require         => [ Class["maverick_gcs::fcs"], Class["nginx"] ],
    }
    
}