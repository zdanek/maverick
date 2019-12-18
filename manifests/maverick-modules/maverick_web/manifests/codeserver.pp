class maverick_web::codeserver (
    $active = true,
    $webport = "6789",
    $basepath = "/srv/maverick",
    $password = "wingman",
    $replace_password = false,
    $vscode_version = "1.41.0",
    $build_type = "production",
) {

    file { [ "/srv/maverick/data/web/codeserver", "/srv/maverick/data/web/codeserver/User", "/srv/maverick/.vscode", "/srv/maverick/software/codeserver", "/etc/systemd/system/maverick-codeserver.service.d"]:
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    } ->
    file { "/srv/maverick/data/web/codeserver/User/settings.json":
        owner   => "mav",
        group   => "mav",
        source  => "puppet:///modules/maverick_web/codeserver-settings.json",
        replace => false,
    } ->
    file { "/srv/maverick/.vscode/settings.json":
        owner   => "mav",
        group   => "mav",
        source  => "puppet:///modules/maverick_web/codeserver-workspace-settings.json",
        replace => false,
    }

    if ! ("install_flag_codeserver" in $installflags) {
        ensure_packages(["libxkbfile-dev", "libsecret-1-dev"])
        oncevcsrepo { "git-codeserver":
            gitsource   => "https://github.com/cdr/code-server",
            dest        => "/srv/maverick/var/build/codeserver",
        } ->
        exec { "codeserver-build":
            path        => ["/bin", "/usr/bin", "/opt/nodejs/bin"],
            command		=> "yarn build ${vscode_version} ${build_type} >/srv/maverick/var/log/build/codeserver.build.log 2>&1",
            cwd		    => "/srv/maverick/var/build/codeserver",
            unless      => "/bin/ls -ld /srv/maverick/var/build/codeserver/build/code-server${build_type}-vsc${vscode_version}*",
            require     => Package['yarn'],
            timeout		=> 0,
            user        => "mav",
        } ->
        exec { "codeserver-package":
            path        => ["/bin", "/usr/bin", "/opt/nodejs/bin"],
            command		=> "yarn binary ${vscode_version} ${build_type} >/srv/maverick/var/log/build/codeserver.package.log 2>&1",
            cwd		    => "/srv/maverick/var/build/codeserver",
            unless      => "/bin/ls -ld /srv/maverick/var/build/codeserver/binaries/code-server${build_type}-vsc${vscode_version}*",
            require     => Package['yarn'],
            timeout		=> 0,
            user        => "mav",
        } ->
        exec { "codeserver-install":
            command     => "/bin/cp /srv/maverick/var/build/codeserver/binaries/code-server${build_type}-vsc${vscode_version}* /srv/maverick/software/codeserver/codeserver",
            creates     => "/srv/maverick/software/codeserver/codeserver",
            user        => "mav",        
        } ->
        exec { "codeserver-install-extensions":
            command     => "/bin/cp -R /srv/maverick/var/build/codeserver/build/code-server${build_type}-vsc${vscode_version}*/extensions /srv/maverick/software/codeserver",
            creates     => "/srv/maverick/software/codeserver/extensions/git/package.json",
            user        => "mav",
            before      => Exec["codeserver-ext-python"],
        } ->
        file { "/srv/maverick/var/build/.install_flag_codeserver":
            ensure      => present,
        }
    }

    # Install some default extensions
    exec { "codeserver-ext-python":
        command     => "/srv/maverick/software/codeserver/codeserver --user-data-dir /srv/maverick/data/web/codeserver --install-extension ms-python.python",
        user        => "mav",
        timeout     => 0,
        unless      => "/bin/ls -ld /srv/maverick/data/web/codeserver/extensions/ms-python.python-*/package.json",
        before      => Service["maverick-codeserver"],
        notify      => Service["maverick-codeserver"],
    } ->
    exec { "codeserver-ext-cplusplus":
        command     => "/srv/maverick/software/codeserver/codeserver --user-data-dir /srv/maverick/data/web/codeserver --install-extension ms-vscode.cpptools",
        user        => "mav",
        timeout     => 0,
        unless      => "/bin/ls -ld /srv/maverick/data/web/codeserver/extensions/ms-vscode.cpptools-*/package.json",
        before      => Service["maverick-codeserver"],
        notify      => Service["maverick-codeserver"],
    } ->
    exec { "codeserver-ext-vscodeicons":
        command     => "/srv/maverick/software/codeserver/codeserver --user-data-dir /srv/maverick/data/web/codeserver --install-extension vscode-icons-team.vscode-icons",
        user        => "mav",
        timeout     => 0,
        unless      => "/bin/ls -ld /srv/maverick/data/web/codeserver/extensions/vscode-icons-team.vscode-icons-*/package.json",
        before      => Service["maverick-codeserver"],
        notify      => Service["maverick-codeserver"],
    } ->
    exec { "codeserver-ext-onedarkpro":
        command     => "/srv/maverick/software/codeserver/codeserver --user-data-dir /srv/maverick/data/web/codeserver --install-extension zhuangtongfa.material-theme",
        user        => "mav",
        timeout     => 0,
        unless      => "/bin/ls -ld /srv/maverick/data/web/codeserver/extensions/zhuangtongfa.material-theme-*/package.json",
        before      => Service["maverick-codeserver"],
        notify      => Service["maverick-codeserver"],
    } ->
    exec { "codeserver-ext-nightowl":
        command     => "/srv/maverick/software/codeserver/codeserver --user-data-dir /srv/maverick/data/web/codeserver --install-extension sdras.night-owl",
        user        => "mav",
        timeout     => 0,
        unless      => "/bin/ls -ld /srv/maverick/data/web/codeserver/extensions/sdras.night-owl-*/package.json",
        before      => Service["maverick-codeserver"],
        notify      => Service["maverick-codeserver"],
    } ->
    exec { "codeserver-ext-gitlens":
        command     => "/srv/maverick/software/codeserver/codeserver --user-data-dir /srv/maverick/data/web/codeserver --install-extension eamodio.gitlens",
        user        => "mav",
        timeout     => 0,
        unless      => "/bin/ls -ld /srv/maverick/data/web/codeserver/extensions/eamodio.gitlens-*/package.json",
        before      => Service["maverick-codeserver"],
        notify      => Service["maverick-codeserver"],
    } ->
    exec { "codeserver-ext-vetur":
        command     => "/srv/maverick/software/codeserver/codeserver --user-data-dir /srv/maverick/data/web/codeserver --install-extension octref.vetur",
        user        => "mav",
        timeout     => 0,
        unless      => "/bin/ls -ld /srv/maverick/data/web/codeserver/extensions/octref.vetur-*/package.json",
        before      => Service["maverick-codeserver"],
        notify      => Service["maverick-codeserver"],
    } ->
    exec { "codeserver-ext-puppet":
        command     => "/srv/maverick/software/codeserver/codeserver --user-data-dir /srv/maverick/data/web/codeserver --install-extension jpogran.puppet-vscode",
        user        => "mav",
        timeout     => 0,
        unless      => "/bin/ls -ld /srv/maverick/data/web/codeserver/extensions/jpogran.puppet-vscode-*/package.json",
        before      => Service["maverick-codeserver"],
        notify      => Service["maverick-codeserver"],
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
        notify      => [ Exec["maverick-systemctl-daemon-reload"], Service["maverick-codeserver"] ],
    } ->
    file { "/etc/systemd/system/maverick-codeserver.service.d/password.conf":
        content     => "[Service]\nEnvironment='PASSWORD=${password}'",
        owner       => "mav",
        group       => "mav",
        mode        => "644",
        replace     => $replace_password,
        notify      => Service["maverick-codeserver"],
    } ->
    service { "maverick-codeserver":
        ensure      => $_ensure,
        enable      => $_enable,
    }
    
    # status.d entry
    file { "/srv/maverick/software/maverick/bin/status.d/120.web/102.codeserver.status":
        owner   => "mav",
        content => "codeserver,CodeServer/VSCode IDE\n",
    }

}
