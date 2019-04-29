class maverick_web::codeserver (
    $active = true,
    $webport = "6795",
    $basepath = "/srv/maverick",
    $password = "wingman",
    $filewatchers = "8192"
) {

    file { "/srv/maverick/data/web/codeserver":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    }

    if ! ("install_flag_codeserver" in $installflags) {
        ensure_packages(["libxkbfile-dev", "libsecret-1-dev"])

        oncevcsrepo { "git-codeserver":
            gitsource   => "https://github.com/codercom/code-server.git",
            dest        => "/srv/maverick/software/codeserver",
        } ->
        exec { "codeserver-preinstall":
            command		=> "/usr/bin/yarn >/srv/maverick/var/log/build/codeserver.preinstall.log 2>&1",
            cwd		    => "/srv/maverick/software/codeserver",
            creates		=> "/srv/maverick/software/codeserver/node_modules/node-pty",
            timeout		=> 0,
            user        => "mav",
            require     => Class["maverick_web::nodejs"],
        } ->
        exec { "codeserver-build":
            command		=> "/usr/bin/yarn task build:server:binary >/srv/maverick/var/log/build/codeserver.build.log 2>&1",
            cwd		    => "/srv/maverick/software/codeserver",
            creates		=> "/srv/maverick/software/codeserver/packages/server/cli-",
            timeout		=> 0,
            user        => "mav",
        } ->
        exec { "codeserver-symlink":
            command     => "/bin/ln -s cli-* code-server",
            cwd         => "/srv/maverick/software/codeserver/packages/server",
            user        => "mav",
            creates     => "/srv/maverick/software/codeserver/packages/server/code-server",
        } ->
        file { "/srv/maverick/var/build/.install_flag_codeserver":
            ensure      => present,
            before      => Exec["codeserver-ext-python"],
        }
    }

    # Increase kernel inotify watcher limits
    base::sysctl::conf { 
        "fs.inotify.max_user_watches": 	value => $filewatchers;
    }

    # Install some default extensions
    exec { "codeserver-ext-python":
        command     => "/srv/maverick/software/codeserver/packages/server/code-server --user-data-dir /srv/maverick/data/web/codeserver --install-extension ms-python.python",
        user        => "mav",
        timeout     => 0,
        unless      => "/bin/ls -ld /srv/maverick/data/web/codeserver/extensions/ms-python.python-*/package.json",
        before      => Service_wrapper["maverick-codeserver"],
        notify      => Service_wrapper["maverick-codeserver"],
    } ->
    exec { "codeserver-ext-cplusplus":
        command     => "/srv/maverick/software/codeserver/packages/server/code-server --user-data-dir /srv/maverick/data/web/codeserver --install-extension ms-vscode.cpptools",
        user        => "mav",
        timeout     => 0,
        unless      => "/bin/ls -ld /srv/maverick/data/web/codeserver/extensions/ms-vscode.cpptools-*/package.json",
        before      => Service_wrapper["maverick-codeserver"],
        notify      => Service_wrapper["maverick-codeserver"],
    } ->
    exec { "codeserver-ext-vscodeicons":
        command     => "/srv/maverick/software/codeserver/packages/server/code-server --user-data-dir /srv/maverick/data/web/codeserver --install-extension vscode-icons-team.vscode-icons",
        user        => "mav",
        timeout     => 0,
        unless      => "/bin/ls -ld /srv/maverick/data/web/codeserver/extensions/vscode-icons-team.vscode-icons-*/package.json",
        before      => Service_wrapper["maverick-codeserver"],
        notify      => Service_wrapper["maverick-codeserver"],
    } ->
    exec { "codeserver-ext-onedarkpro":
        command     => "/srv/maverick/software/codeserver/packages/server/code-server --user-data-dir /srv/maverick/data/web/codeserver --install-extension zhuangtongfa.material-theme",
        user        => "mav",
        timeout     => 0,
        unless      => "/bin/ls -ld /srv/maverick/data/web/codeserver/extensions/zhuangtongfa.material-theme-*/package.json",
        before      => Service_wrapper["maverick-codeserver"],
        notify      => Service_wrapper["maverick-codeserver"],
    } ->
    exec { "codeserver-ext-gitlens":
        command     => "/srv/maverick/software/codeserver/packages/server/code-server --user-data-dir /srv/maverick/data/web/codeserver --install-extension eamodio.gitlens",
        user        => "mav",
        timeout     => 0,
        unless      => "/bin/ls -ld /srv/maverick/data/web/codeserver/extensions/eamodio.gitlens-*/package.json",
        before      => Service_wrapper["maverick-codeserver"],
        notify      => Service_wrapper["maverick-codeserver"],
    } ->
    exec { "codeserver-ext-vetur":
        command     => "/srv/maverick/software/codeserver/packages/server/code-server --user-data-dir /srv/maverick/data/web/codeserver --install-extension octref.vetur",
        user        => "mav",
        timeout     => 0,
        unless      => "/bin/ls -ld /srv/maverick/data/web/codeserver/extensions/octref.vetur-*/package.json",
        before      => Service_wrapper["maverick-codeserver"],
        notify      => Service_wrapper["maverick-codeserver"],
    } ->
    exec { "codeserver-ext-puppet":
        command     => "/srv/maverick/software/codeserver/packages/server/code-server --user-data-dir /srv/maverick/data/web/codeserver --install-extension jpogran.puppet-vscode",
        user        => "mav",
        timeout     => 0,
        unless      => "/bin/ls -ld /srv/maverick/data/web/codeserver/extensions/jpogran.puppet-vscode-*/package.json",
        before      => Service_wrapper["maverick-codeserver"],
        notify      => Service_wrapper["maverick-codeserver"],
    }
    
    
    if defined(Class["::maverick_security"]) {
        maverick_security::firewall::firerule { "codeserver":
            ports       => $webport,
            ips         => lookup("firewall_ips"),
            proto       => "tcp"
        }
    }

    # Control running service
    if $active == true {
        $_ensure = running
        $_enable = true
    } else {
        $_ensure = stopped
        $_enable = false
    }
    file { "/etc/systemd/system/maverick-codeserver.service":
        content     => template("maverick_web/codeserver.service.erb"),
        owner       => "root",
        group       => "root",
        mode        => "644",
        notify      => Exec["maverick-systemctl-daemon-reload"],
    } ->
    service_wrapper { "maverick-codeserver":
        ensure      => $_ensure,
        enable      => $_enable,
    }
    
}
