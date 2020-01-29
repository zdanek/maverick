# @summary
#   Maverick_analysis::Collect class
#   This class installs/manages collectd software (collectd.org), which is used to log system metrics.
#
# @example Declaring the class
#   This class is included from maverick_analysis class and should not be included from elsewhere
#
# @param active
#   If true, set the maverick-collectd service to running and enabled (at boot).
# @param install_type
#   If 'source', then compile collectd from git source.  Should always be set to source as it installs into custom location (~/software/collectd).
# @param git_source
#   Github repo location to clone source code from.
# @param git_revision
#   Github branch to compile.
#
class maverick_analysis::collect (
    Boolean $active = true,
    Enum['source', 'binary'] $install_type = "source",
    String $git_source = "https://github.com/collectd/collectd.git",
    String $git_revision = "5.10.0",
) {
    # Install from source
    if $install_type == "source" {
        $manage_package = false
        $manage_repo = false
        # Ensure build dependencies are installed
        ensure_packages(["build-essential", "autoconf", "automake", "libtool"])
        ensure_packages(["collectd", "collectd-core"], {'ensure'=>'absent'})
        unless "install_flag_collectd" in $installflags {
            ensure_packages(["flex", "bison", "libopenipmi-dev", "libsensors4-dev", "libsnmp-dev"])
            oncevcsrepo { "git-collectd":
                gitsource   => $git_source,
                revision    => $git_revision,
                dest        => "/srv/maverick/var/build/collectd",
            } ->
            exec { "collectd-build.sh":
                user        => "mav",
                command     => "/srv/maverick/var/build/collectd/build.sh",
                cwd         => "/srv/maverick/var/build/collectd",
                creates     => "/srv/maverick/var/build/collectd/configure",
                timeout     => 0,
                require     => [ Package["libopenipmi-dev"], Package["libsensors4-dev"] ],
            } ->
            exec { "collectd-configure":
                user        => "mav",
                command     => "/srv/maverick/var/build/collectd/configure --disable-werror --prefix=/srv/maverick/software/collectd >/srv/maverick/var/log/build/collectd-configure.log 2>&1",
                cwd         => "/srv/maverick/var/build/collectd",
                creates     => "/srv/maverick/var/build/collectd/Makefile",
                timeout     => 0,
            } ->
            exec { "collectd-make":
                user        => "mav",
                command     => "/usr/bin/make >/srv/maverick/var/log/build/collectd-make.log 2>&1",
                cwd         => "/srv/maverick/var/build/collectd",
                creates     => "/srv/maverick/var/build/collectd/src/daemon/collectd",
                timeout     => 0,
            } ->
            exec { "collectd-install":
                user        => "mav",
                command     => "/usr/bin/make install >/srv/maverick/var/log/build/collectd-install 2>&1",
                cwd         => "/srv/maverick/var/build/collectd",
                creates     => "/srv/maverick/software/collectd/sbin/collectd",
                timeout     => 0,
            } ->
            file { "/srv/maverick/var/build/.install_flag_collectd":
                owner           => "mav",
                group           => "mav",
                ensure          => present,
            }
        }
        file { "/etc/systemd/system/maverick-collectd.service":
            ensure          => present,
            owner           => "root",
            group           => "root",
            mode            => "644",
            source          => "puppet:///modules/maverick_analysis/maverick-collectd-source.service",
            notify          => Exec["maverick-systemctl-daemon-reload"],
            before          => Class["collectd"],
        }
        file { "/srv/maverick/config/analysis/collectd":
            ensure          => directory,
            mode            => "755",
            owner           => "mav",
            group           => "mav",
        }
        $config_file = "/srv/maverick/config/analysis/collectd/collectd.conf"
        $plugin_conf_dir = "/srv/maverick/config/analysis/collectd/conf.d"
        $collectd_dir = '/srv/maverick/software/collectd'
        $typesdb = "${collectd_dir}/share/collectd/types.db"
    } else {
        $manage_package = true
        file { "/etc/systemd/system/maverick-collectd.service":
            ensure          => present,
            owner           => "root",
            group           => "root",
            mode            => "644",
            source          => "puppet:///modules/maverick_analysis/maverick-collectd-dpkg.service",
            notify          => Exec["maverick-systemctl-daemon-reload"],
            before          => Class["collectd"],
        }
        $collectd_dir = '/etc/collectd'
        $config_file = "${collectd_dir}/collectd.conf"
        $plugin_conf_dir = "${collectd_dir}/conf.d"
        $typesdb = "/usr/share/collectd/types.db"

        # Collectd repos only provide i386/amd64 packages
        if $::architecture == "i386" or $::architecture == "amd64" and ($operatingsystem == "Ubuntu" and versioncmp($::operatingsystemrelease, "17.04") < 0) {
            $manage_repo = true
        } else {
            $manage_repo = false
        }
    }

    if $active == true {
        $service_ensure = running
        $service_enable = true
    } else {
        $service_ensure = stopped
        $service_enable = false
    }
    class { "collectd":
        config_file     => $config_file,
        plugin_conf_dir => $plugin_conf_dir,
        plugin_conf_dir_mode => "755",
        purge           => true,
        recurse         => true,
        purge_config    => true,
        minimum_version => '5.7',
        manage_package  => $manage_package,
        manage_repo     => $manage_repo,
        manage_service  => true,
        service_ensure  => $service_ensure,
        service_enable  => $service_enable,
        service_name    => 'maverick-collectd',
        typesdb         => [$typesdb],
    }
    service { "collectd":
        ensure          => stopped,
        enable          => false,
    }

    ### Collectd Plugins
    collectd::plugin::aggregation::aggregator {'cpu':
        plugin           => 'cpu',
        agg_type         => 'percent',
        groupby          => ["Host", "TypeInstance",],
        calculatesum     => true,
        calculateaverage => true,
    }
    class { 'collectd::plugin::contextswitch': }
    class { 'collectd::plugin::cpu':
        reportbystate => true,
        reportbycpu => true,
        valuespercentage => true,
    }
    class { 'collectd::plugin::cpufreq': }
    class { 'collectd::plugin::df':
        mountpoints    => [],
        fstypes        => ['nfs','tmpfs','autofs','gpfs','proc','devpts'],
        ignoreselected => true,
    }
    class { 'collectd::plugin::disk':
        disks           => ['/^dm/'],
        ignoreselected => true,
        #udevnameattr   => 'DM_NAME',
    }
    class { 'collectd::plugin::interface':
        interfaces     => ['lo'],
        ignoreselected => true
    }
    /*
    class { 'collectd::plugin::ipmi':
        # ignore_selected           => true,
        # sensors                   => ['temperature'],
        notify_sensor_add         => true,
        notify_sensor_remove      => true,
        notify_sensor_not_present => true,
    }
    */
    class { 'collectd::plugin::irq': }
    class { 'collectd::plugin::load': }
    class { 'collectd::plugin::memory': }
    class { 'collectd::plugin::network':
        maxpacketsize => '1452',
    }
    class { 'collectd::plugin::processes': }
    class { 'collectd::plugin::protocols':
        values => ['/^Tcp:*/', '/^Udp:*/',],
        ignoreselected => false,
    }
    class { 'collectd::plugin::sensors': }
    class { 'collectd::plugin::swap':
        reportbydevice => false,
        reportbytes    => true
    }
    class { 'collectd::plugin::thermal': }
    class { 'collectd::plugin::uptime': }
    class { 'collectd::plugin::users': }
    # This collects a lot of data which we don't really need
    #class { 'collectd::plugin::vmem':
    #    verbose => true,
    #}

    # Configure an exec plugin to run power script on Intel RAPL platform, to retrieve power consumption
    if $rapl_present == "yes" {
        ensure_packages(["bc"])
        collectd::plugin::exec::cmd { 'rapl-power':
            user => mav,
            group => mav,
            exec => ["/srv/maverick/software/maverick/manifests/maverick-modules/maverick_analysis/files/rapl-power.sh"],
        }
    }

    # Configure an exec plugin to run power script on Nvidia Tegra platform, to retrieve power consumption
    if $tegra_present == "yes" {
        ensure_packages(["bc"])
        # Hack mav user into i2c group so it can access the INA i2c device
        exec { "mav-i2c-group":
            command     => "/usr/sbin/usermod -a -G i2c mav",
            unless      => "/bin/grep i2c /etc/group |grep mav",
        } ->
        collectd::plugin::exec::cmd { 'tegra-power':
            user        => "mav",
            group       => "i2c",
            exec        => ["/srv/maverick/software/maverick/manifests/maverick-modules/maverick_analysis/files/tegra-power.sh"],
            notify      => Service["maverick-collectd"],
        }
    }

    # status.d entry for collectd
    file { "/srv/maverick/software/maverick/bin/status.d/121.analysis/103.collectd.status":
        owner   => "mav",
        content => "collectd,System Metrics Collector\n",
    }
}
