class maverick_mavlink (
    $cmavnode_install = true,
    $cmavnode_source = "https://github.com/MonashUAS/cmavnode.git",
    $mavlink_router_install = true,
    $mavlink_router_source = "https://github.com/01org/mavlink-router.git",
    $mavproxy_install = true,
    $mavproxy_source = "https://github.com/ArduPilot/MAVProxy.git",
    $mavproxy_type = "pip",
    $dronekit_install = true,
    $dronekit_la_install = true,
    $dronekit_la_source = "https://github.com/dronekit/dronekit-la.git",
    $mavcesium_apikey = "",
    $mavcesium_ports = [6790],
    $mavcesium_source = "https://github.com/SamuelDudley/MAVCesium.git",
) {
    
    $buildparallel = ceiling((0 + $::processorcount) / 2) # Restrict build parallelization to roughly processors/2 (to restrict memory usage during compilation)
    
    # Create config directory for all mavlink configs
    file { "/srv/maverick/data/config/mavlink":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    }
    # Create data directory for mavlink
    file { "/srv/maverick/data/mavlink":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    }
    
    ### cmavnode
    # Install cmavnode from gitsource
    if $cmavnode_install {
        if ! ("install_flag_cmavnode" in $installflags) {
            ensure_packages(["libboost-all-dev", "cmake", "libconfig++-dev", "libreadline-dev"])
            oncevcsrepo { "git-cmavnode":
                gitsource   => $cmavnode_source,
                dest        => "/srv/maverick/var/build/cmavnode",
                submodules  => true,
            } ->
            # Create build directory
            file { "/srv/maverick/var/build/cmavnode/build":
                ensure      => directory,
                owner       => "mav",
                group       => "mav",
                mode        => 755,
            }
            exec { "cmavnode-prepbuild":
                user        => "mav",
                timeout     => 0,
                command     => "/usr/bin/cmake -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/cmavnode ..",
                cwd         => "/srv/maverick/var/build/cmavnode/build",
                creates     => "/srv/maverick/var/build/cmavnode/build/Makefile",
                require     => [ File["/srv/maverick/var/build/cmavnode/build"], Package["libreadline-dev"] ], # ensure we have all the dependencies satisfied
            } ->
            exec { "cmavnode-build":
                user        => "mav",
                timeout     => 0,
                command     => "/usr/bin/make -j${buildparallel} >/srv/maverick/var/log/build/cmavnode.build.out 2>&1",
                cwd         => "/srv/maverick/var/build/cmavnode/build",
                creates     => "/srv/maverick/var/build/cmavnode/build/cmavnode",
                require     => Exec["cmavnode-prepbuild"],
            } ->
            exec { "cmavnode-install":
                user        => "mav",
                timeout     => 0,
                command     => "/usr/bin/make install >/srv/maverick/var/log/build/cmavnode.install.out 2>&1",
                cwd         => "/srv/maverick/var/build/cmavnode/build",
                creates     => "/srv/maverick/software/cmavnode/bin/cmavnode",
            }
            file { "/srv/maverick/var/build/.install_flag_cmavnode":
                ensure      => file,
                owner       => "mav",
            }
        }
        file { "/etc/systemd/system/maverick-cmavnode@.service":
            source      => "puppet:///modules/maverick_mavlink/maverick-cmavnode@.service",
            owner       => "root",
            group       => "root",
            mode        => 644,
            notify      => Exec["maverick-systemctl-daemon-reload"],
        }
        file { "/srv/maverick/software/maverick/bin/cmavnode.sh":
            ensure      => link,
            target      => "/srv/maverick/software/maverick/manifests/maverick-modules/maverick_mavlink/files/cmavnode.sh",
        }
    }

    ### mavlink-router
    # Install mavlink-router from gitsource
    if $mavlink_router_install {
        if ! ("install_flag_mavlink-router" in $installflags) {
            oncevcsrepo { "git-mavlink-router":
                gitsource   => $mavlink_router_source,
                dest        => "/srv/maverick/var/build/mavlink-router",
                submodules  => true,
            } ->
            exec { "mavlink-router-build":
                user        => "mav",
                timeout     => 0,
                command     => "/srv/maverick/var/build/mavlink-router/autogen.sh && CFLAGS='-g -O2' /srv/maverick/var/build/mavlink-router/configure --with-dialect=ardupilotmega --prefix=/srv/maverick/software/mavlink-router && /usr/bin/make -j${buildparallel} && make install >/srv/maverick/var/log/build/mavlink-router.build.out 2>&1",
                cwd         => "/srv/maverick/var/build/mavlink-router",
                creates     => "/srv/maverick/software/mavlink-router/bin/mavlink-routerd",
            } ->
            file { "/srv/maverick/var/build/.install_flag_mavlink-router":
                ensure      => file,
                owner       => "mav",
            }
        }
        file { "/etc/systemd/system/maverick-mavlink-router@.service":
            source      => "puppet:///modules/maverick_mavlink/maverick-mavlink-router@.service",
            owner       => "root",
            group       => "root",
            mode        => 644,
            notify      => Exec["maverick-systemctl-daemon-reload"],
        }
    }
    
    
    ### Mavproxy
    # Install mavproxy globally (not in virtualenv) from pip
    if $mavproxy_install and $mavproxy_type == "pip" {
        ensure_packages(["python-lxml", "libxml2-dev", "libxslt1-dev"])
        install_python_module { 'pip-mavproxy-global':
            pkgname     => 'mavproxy',
            ensure      => present,
            timeout     => 0,
            require     => Package["python-lxml", "libxml2-dev", "libxslt1-dev"],
        } ->
        file { "/etc/systemd/system/maverick-mavproxy@.service":
            source      => "puppet:///modules/maverick_mavlink/maverick-mavproxy@.service",
            owner       => "root",
            group       => "root",
            mode        => 644,
            notify      => Exec["maverick-systemctl-daemon-reload"],
        } ->
        file { "/srv/maverick/software/maverick/bin/mavproxy.sh":
            ensure      => link,
            target      => "/srv/maverick/software/maverick/manifests/maverick-modules/maverick_mavlink/files/mavproxy.sh",
        }
    }

    # Install mavproxy from source
    if $mavproxy_install and $mavproxy_type == "source" {
        ensure_packages(["python-lxml", "libxml2-dev", "libxslt1-dev"])
        if ! ("install_flag_mavproxy" in $installflags) {
            # First uninstall pip mavproxy, to remove any conflicts
            install_python_module { 'mavproxy':
                pkgname     => 'mavproxy',
                ensure      => 'absent',
            } ->
            oncevcsrepo { "git-mavproxy":
                gitsource   => $mavproxy_source,
                dest        => "/srv/maverick/var/build/mavproxy",
                submodules  => true,
            } ->
            exec { "mavproxy-build":
                command     => "/usr/bin/python setup.py build install --user",
                cwd         => "/srv/maverick/var/build/mavproxy",
                user        => "root",
                timeout     => 0,
                #creates     => "",
            } ->
            file { "/srv/maverick/var/build/.install_flag_mavproxy":
                ensure      => file,
                owner       => "mav",
                group       => "mav",
                mode        => "644",
            }
        }
        file { "/etc/systemd/system/maverick-mavproxy@.service":
            source      => "puppet:///modules/maverick_mavlink/maverick-mavproxy@.service",
            owner       => "root",
            group       => "root",
            mode        => 644,
            notify      => Exec["maverick-systemctl-daemon-reload"],
        } ->
        file { "/srv/maverick/software/maverick/bin/mavproxy.sh":
            ensure      => link,
            target      => "/srv/maverick/software/maverick/manifests/maverick-modules/maverick_mavlink/files/mavproxy.sh",
        }
    }

    ## Install mavcesium dependencies and services
    # Overlay mavcesium master ontop of mavproxy installed version
    exec { "remove-mavproxy-mavcesium":
        command     => "/bin/rm -rf /usr/local/lib/python2.7/dist-packages/MAVProxy/modules/mavproxy_cesium",
        unless      => "/bin/ls /usr/local/lib/python2.7/dist-packages/MAVProxy/modules/mavproxy_cesium/.git",
        require     => File["/srv/maverick/software/maverick/bin/mavproxy.sh"],
    } ->
    oncevcsrepo { "git-mavcesium":
        gitsource   => $mavcesium_source,
        dest        => "/usr/local/lib/python2.7/dist-packages/MAVProxy/modules/mavproxy_cesium",
        submodules  => true,
    } ->
    install_python_module { "mav-flask":
        pkgname     => "Flask",
        ensure      => present,
        timeout     => 0,
    } ->
    install_python_module { "mav-twisted":
        pkgname     => "twisted",
        ensure      => present,
        timeout     => 0,
    } ->
    install_python_module { "mav-autobahn":
        pkgname     => "autobahn",
        ensure      => present,
        timeout     => 0,
    } ->
    install_python_module { "mav-configparser":
        pkgname     => "configparser",
        ensure      => present,
        timeout     => 0,
    } ->
    file { "/usr/local/lib/python2.7/dist-packages/MAVProxy/modules/mavproxy_cesium/app/api_keys.txt":
        content     => "{\"bing\": \"${mavcesium_apikey}\"}",
        mode        => "644",
        owner       => "mav",
    } ->
    file { "/usr/local/lib/python2.7/dist-packages/MAVProxy/modules/mavproxy_cesium/app/mavcesium_default.ini":
        owner       => "mav",
        mode        => "644",
    } ->
    file { "/srv/maverick/data/config/mavlink/cesium":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    } ->
    file { "/srv/maverick/data/config/mavlink/cesium/api_keys.txt":
        ensure      => link,
        target      => "/usr/local/lib/python2.7/dist-packages/MAVProxy/modules/mavproxy_cesium/app/api_keys.txt",
    } ->
    file { "/srv/maverick/data/config/mavlink/cesium/mavcesium_default.ini":
        ensure      => link,
        target      => "/usr/local/lib/python2.7/dist-packages/MAVProxy/modules/mavproxy_cesium/app/mavcesium_default.ini",
    }
    
    if defined(Class["::maverick_security"]) {
        maverick_security::firewall::firerule { "mavcesium":
            ports       => $mavcesium_ports,
            ips         => hiera("all_ips"),
            proto       => "tcp"
        }
    }
    
    # Install dronekit globally (not in virtualenv) from pip
    if $dronekit_install {
        install_python_module { "pip-dronekit-global":
            pkgname     => "dronekit",
            ensure      => "present",
            timeout     => 0,
        }
    }

    if ! ("install_flag_dronekit-la" in $installflags) {
        # Install dronekit-la (log analyzer)
        if $dronekit_la_install {
            oncevcsrepo { "git-dronekit-la":
                gitsource   => $dronekit_la_source,
                dest        => "/srv/maverick/var/build/dronekit-la",
                submodules  => true,
                owner        => mav,
                group       => mav,
            } ->
            # Compile dronekit-la
            exec { "compile-dronekit-la":
                command     => "/usr/bin/make -j${::processorcount} dronekit-la dataflash_logger >/srv/maverick/var/log/build/dronekit-la.build.log 2>&1",
                creates     => "/srv/maverick/var/build/dronekit-la/dronekit-la",
                user        => "mav",
                cwd         => "/srv/maverick/var/build/dronekit-la",
                timeout     => 0,
            } ->
            file { "/srv/maverick/software/dronekit-la":
                ensure      => directory,
                owner       => "mav",
                group       => "mav",
            } ->
            exec { "install-dronekit-la":
                command     => "/bin/cp dronekit-la dataflash_logger README.md /srv/maverick/software/dronekit-la; chmod a+rx /srv/maverick/software/dronekit-la/* >/srv/maverick/var/log/build/dronekit-la.install.log 2>&1",
                creates     => "/srv/maverick/software/dronekit-la/dronekit-la",
                user        => "mav",
                cwd         => "/srv/maverick/var/build/dronekit-la",
                timeout     => 0,
            } ->
            file { "/srv/maverick/var/build/.install_flag_dronekit-la":
                ensure      => file,
                owner       => "mav",
            }
        }
    } else {
        # This is only here for other manifests to use as a require
        exec { "install-dronekit-la":
            command     => "/bin/ls",
            refreshonly => true,
        }
    }
    
}