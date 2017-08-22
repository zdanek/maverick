class maverick_web::nginx (
    $port,
    $sslport,
) {
    
    # Nginx doesn't have repo for ARM, so don't try to use it
    if $::architecture =~ "arm" {
        $manage_repo = false
    } else {
        $manage_repo = true
    }
    
    service_wrapper { "apache2":
        ensure      => stopped,
        enable      => false,
    } ->
    class { 'nginx':
        confd_purge     => true,
        server_purge    => true,
        manage_repo     => $manage_repo,
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