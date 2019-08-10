class maverick_analysis::mavlogd (
    $active = true,
    $grafana_port = $maverick_analysis::grafana::webport,
    $grafana_host = $maverick_analysis::grafana::host,
    $grafana_adminpass = $maverick_analysis::grafana::admin_password,
) {
    ensure_packages(["python-lockfile"])
    ensure_packages(["python-daemon"])
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
        #source          => "puppet:///modules/maverick_analysis/maverick-mavlogd.conf",
        content         => template("maverick_analysis/maverick-mavlogd.conf.erb"),
        notify          => Service["maverick-mavlogd"],
    } ->
    file { "/etc/systemd/system/maverick-mavlogd.service":
        ensure          => present,
        mode            => "644",
        owner           => "mav",
        group           => "mav",
        source          => "puppet:///modules/maverick_analysis/maverick-mavlogd.service",
        notify          => Exec["maverick-systemctl-daemon-reload"],
        require         => [Service["maverick-influxd"], Service["maverick-grafana"]],
        before          => Service["maverick-mavlogd"],
    }
    
    if $active == true {
        service{ "maverick-mavlogd":
            ensure          => running,
            enable          => true,
            require         => Package["python-lockfile"],
        }
    } else {
        service{ "maverick-mavlogd":
            ensure          => stopped,
            enable          => false,
            require         => Package["python-lockfile"],
        }
    }

    if defined(Class["::maverick_web"]) {
        # Add a temporary service to run mavlog uploader
        # Note the paths and webport are hardcoded into /srv/maverick/software/maverick-fcs/file_upload.py
        install_python_module { "uploader-tornado":
            pkgname     => "tornado",
            ensure      => atleast,
            version     => "4.5.2",
        } ->
        file { "/etc/systemd/system/maverick-uploader.service":
            ensure          => present,
            mode            => "644",
            owner           => "mav",
            group           => "mav",
            source          => "puppet:///modules/maverick_analysis/maverick-uploader.service",
            notify          => Exec["maverick-systemctl-daemon-reload"],
        }
        
        if $active == true {
            service { "maverick-uploader":
                ensure          => running,
                enable          => true,
                require         => File["/etc/systemd/system/maverick-uploader.service"],
            } ->
            nginx::resource::location { "web-analysis-uploader":
                location    => "/analysis/uploader/",
                proxy       => 'http://localhost:6792/',
                server      => getvar("maverick_web::server_fqdn"),
                require     => [ Class["maverick_gcs::fcs"], Class["nginx"], Service["maverick-uploader"], ],
            }
        } else {
            service { "maverick-uploader":
                ensure      => stopped,
                enable      => false,
            }
        }
    }
}