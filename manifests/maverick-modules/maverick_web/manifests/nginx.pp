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
    
    # Make sure nginx and apache system services are stopped
    service_wrapper { "apache2":
        ensure      => stopped,
        enable      => false,
    } ->
    service_wrapper { "system-nginx":
        service_name    => "nginx",
        ensure          => stopped,
        enable          => false,
    } ->
    file { "/etc/systemd/system/maverick-nginx.service":
        owner       => "root",
        group       => "root",
        mode        => "644",
        source      => "puppet:///modules/maverick_web/maverick-nginx.service",
        notify      => Exec["maverick-systemctl-daemon-reload"],
    } ->
    class { 'nginx':
        confd_purge     => true,
        server_purge    => true,
        manage_repo     => $manage_repo,
        service_manage  => true,
        service_name    => "maverick-nginx",
    }
    nginx::resource::server { "${::hostname}.local":
        listen_port => $port,
        www_root    => '/srv/maverick/software/maverick-fcs/public',
        require     => [ Class["maverick_gcs::fcs"], Service_wrapper["system-nginx"] ],
    }

}