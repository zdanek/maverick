class maverick_analysis::mavlogd (
    $active = true,
) {
    
    install_python_module { 'pip-pyinotify':
        pkgname     => 'pyinotify',
        ensure      => present,
    } ->
    file { "/srv/maverick/software/maverick/bin/maverick-mavlogd":
        ensure          => link,
        target          => "/srv/maverick/software/maverick/manifests/maverick-modules/maverick_analysis/files/maverick-mavlogd",
    } ->
    file { ["/srv/maverick/data/mavlink/inbox", "/srv/maverick/data/mavlink/archive", "/srv/maverick/data/mavlink/archive/inbox", "/srv/maverick/data/mavlink/archive/fc", "/srv/maverick/data/mavlink/archive/sitl"]:
        mode            => "755",
        owner           => "mav",
        group           => "mav",
        ensure          => directory,
    } ->
    file { "/srv/maverick/data/config/analysis/maverick-mavlogd.conf":
        ensure          => file,
        mode            => "644",
        owner           => "mav",
        group           => "mav",
        replace         => false,
        source          => "puppet:///modules/maverick_analysis/maverick-mavlogd.conf",
    } ->
    file { "/etc/systemd/system/maverick-mavlogd.service":
        ensure          => present,
        mode            => "644",
        owner           => "mav",
        group           => "mav",
        source          => "puppet:///modules/maverick_analysis/maverick-mavlogd.service",
        notify          => Exec["maverick-systemctl-daemon-reload"],
        require         => [Service_wrapper["maverick-influxd"], Service["maverick-grafana"]],
    } ->
    service_wrapper{ "maverick-mavlogd":
        ensure          => running,
        enable          => true,
    }
    
}