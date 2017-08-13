class maverick_web::nginx (
    $port,
    $sslport,
) {
    
    service_wrapper { "apache2":
        ensure      => stopped,
        enable      => false,
    } ->
    class { 'nginx':
    }
    nginx::resource::server { "${::hostname}.local":
        listen_port => $port,
        www_root    => '/srv/maverick/software/maverick-fcs/public',
        require     => Class["maverick_gcs::fcs"],
    }
    nginx::resource::location { "web-analysis-graphs":
        location    => "/analysis/grafana/",
        proxy       => 'http://localhost:6790/',
        server      => "${::hostname}.local",
    }
}