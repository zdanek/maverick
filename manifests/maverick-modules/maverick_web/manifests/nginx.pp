class maverick_web::nginx (
    $port,
    $ssl_port,
) {
    
    # Nginx doesn't have repo for ARM, so don't try to use it
    if $::architecture =~ "arm" {
        $manage_repo = false
    } else {
        if $::operatingsystem == "Ubuntu" and versioncmp($::operatingsystemmajrelease, "18") < 0 {
            $manage_repo = true
        } else {
            $manage_repo = false
        }
    }

    # Workaround for ubilinux
    if $::lsbdistid == "ubilinux" and $::lsbdistcodename == "dolcetto" {
        $_release = "stretch"
    } else {
        $_release = $::lsbdistcodename
    }
    
    # Make sure apache system services are stopped
    service_wrapper { "apache2":
        ensure      => stopped,
        enable      => false,
    } ->
    # Make sure nginx system service is stopped
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
        repo_release    => $_release,
        service_manage  => true,
        service_name    => "maverick-nginx",
    }

    nginx::resource::server { "${::hostname}.local":
        listen_port => $port,
        ssl         => true,
        ssl_port    => $ssl_port,
        ssl_cert    => "/srv/maverick/data/web/ssl/${::hostname}.local-webssl.crt",
        ssl_key     => "/srv/maverick/data/web/ssl/${::hostname}.local-webssl.key",
        www_root    => '/srv/maverick/software/maverick-fcs/public',
        require     => [ Class["maverick_gcs::fcs"], ],
        notify      => Service["maverick-nginx"],
    }
    
    nginx::resource::location { "mavca":
        ensure          => present,
        ssl             => true,
        location        => "/security/mavCA.crt",
        location_alias  => "/srv/maverick/data/security/ssl/ca/mavCA.pem",
        index_files     => [],
        server          => "${::hostname}.local",
        require         => [ Class["maverick_gcs::fcs"], ],
        notify          => Service["maverick-nginx"],
    }

    # Add a location to stub stats - used by collectd to collect nginx metrics
    $local_config = {
        'access_log' => 'off',
        'allow'      => '127.0.0.1',
        'deny'       => 'all'
    }
    nginx::resource::location { "status":
        ensure              => present,
        location            => "/nginx_status",
        stub_status         => true,
        location_cfg_append => $local_config,
        server              => "${::hostname}.local",
        notify              => Service["maverick-nginx"],
    }
    
    if defined(Class["maverick_analysis::collect"]) {
        class { 'collectd::plugin::nginx':
          url      => 'http://localhost/nginx_status?auto',
        }
    }

}
