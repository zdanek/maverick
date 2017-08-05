class maverick_web::apache (
    $port,
    $sslport,
) {
    
    service_wrapper { "nginx":
        ensure      => stopped,
        enable      => false,
    } ->
    class { 'apache':
        default_vhost => false,
    } ->
    apache::vhost { "${::hostname}.local":
        port        => $port,
        docroot     => '/srv/maverick/software/maverick-fcs/public',
        require     => Class["maverick_gcs::fcs"],
    }
    
}