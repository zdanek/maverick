# @summary
#   Maverick_analysis::Mavlogd class
#   This class installs/manages Mavlogd, which is a bespoke script from Maverick that is used to import flight data into influxd.
#
# @example Declaring the class
#   This class is included from maverick_analysis class and should not be included from elsewhere
#
# @param active
#   If true, set the maverick-mavlogd service to running and enabled (at boot).
# @param grafana_port
#   The port used to communicate with Grafana.  By default taken from maverick_analysis::grafana::webport and should not be set.
# @param grafana_host
#   The hostname used to communicate with Grafana.  By default taken from maverick_analysis::grafana::host and should not be set.
# @param grafana_adminpass
#   The admin password used to communicate with Grafana.  By default taken from maverick_analysis::grafana::admin_password and should not be set.
#
class maverick_analysis::mavlogd (
    Boolean $active = true,
    Integer $influx_port = $maverick_analysis::influx::http_port,
    Integer $grafana_port = $maverick_analysis::grafana::port,
    String $grafana_host = $maverick_analysis::grafana::host,
    String $grafana_adminpass = $maverick_analysis::grafana::admin_password,
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
        # Note the paths and webport are hardcoded into maverick_analysis/files/file_upload.py
        file { "/etc/systemd/system/maverick-uploader.service":
            ensure          => present,
            mode            => "644",
            owner           => "mav",
            group           => "mav",
            source          => "puppet:///modules/maverick_analysis/maverick-uploader.service",
            notify          => [ Exec["maverick-systemctl-daemon-reload"], Service["maverick-uploader"] ],
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
                require     => [ Class["nginx"], Service["maverick-uploader"], ],
            }
        } else {
            service { "maverick-uploader":
                ensure      => stopped,
                enable      => false,
            }
        }
    }

    # status.d entry
    file { "/srv/maverick/software/maverick/bin/status.d/121.analysis/104.mavlogd.status":
        owner   => "mav",
        content => "mavlogd,Maverick Log Importer\nuploader,Maverick Log Uploader\n",
    }
}
