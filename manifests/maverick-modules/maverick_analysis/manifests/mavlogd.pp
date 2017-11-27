class maverick_analysis::mavlogd (
    $active = true,
) {
    ensure_packages(["python-lockfile", "python-daemon"])
    install_python_module { 'pip-pyinotify':
        pkgname     => 'pyinotify',
        ensure      => present,
    } ->
    install_python_module { 'pip-grafanalib':
        pkgname     => 'grafanalib',
        ensure      => present,
    } ->
    install_python_module { 'pip-grafana_api_client':
        pkgname     => 'grafana-api-client',
        ensure      => present,
    } ->
    install_python_module { 'pip-mavlogd-pymavlink':
        pkgname     => 'pymavlink',
        ensure      => atleast,
        version     => "2.2.5",
        timeout     => 0,
    } ->
    file { "/srv/maverick/software/maverick/bin/maverick-mavlogd":
        ensure          => link,
        target          => "/srv/maverick/software/maverick/manifests/maverick-modules/maverick_analysis/files/maverick-mavlogd",
    } ->
    file { ["/srv/maverick/data/analysis/inbox", "/srv/maverick/data/analysis/anonybox", "/srv/maverick/data/analysis/archive", "/srv/maverick/data/analysis/archive/inbox"]:
        mode            => "755",
        owner           => "mav",
        group           => "mav",
        ensure          => directory,
    } ->
    file { "/srv/maverick/config/analysis/maverick-mavlogd.conf":
        ensure          => file,
        mode            => "644",
        owner           => "mav",
        group           => "mav",
        #replace         => false,
        source          => "puppet:///modules/maverick_analysis/maverick-mavlogd.conf",
        notify          => Service_wrapper["maverick-mavlogd"],
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
        require         => Package["python-lockfile"],
    }
    
    # Add a temporary service to run mavlog uplaoder
    # Note the paths and webport are hardcoded into /srv/maverick/software/maverick-fcs/file_upload.py
    file { "/etc/systemd/system/maverick-uploader.service":
        ensure          => present,
        mode            => "644",
        owner           => "mav",
        group           => "mav",
        source          => "puppet:///modules/maverick_analysis/maverick-uploader.service",
        notify          => Exec["maverick-systemctl-daemon-reload"],
    } ->
    service_wrapper{ "maverick-uploader":
        ensure          => running,
        enable          => true,
    } ->
    nginx::resource::location { "web-analysis-uploader":
        location    => "/analysis/uploader/",
        proxy       => 'http://localhost:6792/',
        server      => "${::hostname}.local",
        require     => [ Class["maverick_gcs::fcs"], Class["nginx"] ],
    }

}