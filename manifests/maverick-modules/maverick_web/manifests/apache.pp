class maverick_web::apache (
    $port,
    $ssl_port,
    $server_hostname = $maverick_web::server_fqdn,
) {
    
    service { "nginx":
        ensure      => stopped,
        enable      => false,
    } ->
    class { 'apache':
        default_vhost => false,
    } ->
    apache::vhost { $server_hostname:
        port        => $port,
        docroot     => '/srv/maverick/software/maverick-fcs/public',
        require     => Class["maverick_gcs::fcs"],
    }
    
}