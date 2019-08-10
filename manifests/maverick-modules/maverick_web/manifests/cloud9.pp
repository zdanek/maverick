class maverick_web::cloud9 (
    $cloud9_active = true,
    $webport = "6789",
    $basepath = "/srv/maverick",
    $cloud9_password = "wingman",
) {
    if $cloud9_installed == "no" {
        warning("Cloud9 will be compiled and can take a long time, please be patient..")
    }
    
    if $cloud9_active == true {
        $_ensure = running
        $_enable = true
    } else {
        $_ensure = stopped
        $_enable = false
    }
    
    # Install system ncurses first, so cloud9 doesn't have to compile it
    ensure_packages(["libncurses5", "libncurses5-dev", "tmux"])

    oncevcsrepo { "git-cloud9":
        gitsource   => "https://github.com/c9/core.git",
        dest        => "/srv/maverick/software/cloud9",
        notify		=> Exec["install-cloud9"],
        #revision    => "c5d3f33749439f90b4e3f40d18854f1fde31b13a",
        depth       => "",
    } ->
    file { "/srv/maverick/software/cloud9/scripts/maverick.c9.install-sdk.sh":
        source      => "puppet:///modules/maverick_web/maverick.c9.install-sdk.sh",
        mode        => "755",
        owner       => "mav",
        group       => "mav",
    } ->
    file { "/srv/maverick/software/cloud9/scripts/maverick.c9.install.sh":
        source      => "puppet:///modules/maverick_web/maverick.c9.install.sh",
        mode        => "755",
        owner       => "mav",
        group       => "mav",
    } ->
    exec { "install-cloud9":
        command		=> "/srv/maverick/software/cloud9/scripts/maverick.c9.install-sdk.sh >/srv/maverick/var/log/build/cloud9-sdk.build.log 2>&1",
        cwd		    => "/srv/maverick/software/cloud9",
        creates		=> [ "/srv/maverick/software/cloud9/node_modules/.gitignore", "/srv/maverick/.c9/node_modules" ],
        timeout		=> 0,
        user        => "mav",
        environment => ["HOME=/srv/maverick"],
        require     => Class["maverick_web::nodejs"],
    } ->
    exec { "reset-cloud9":
        command     => "/usr/bin/git reset --hard",
        cwd         => "/srv/maverick/software/cloud9",
        creates     => "/srv/maverick/software/cloud9/node_modules/treehugger",
        user        => "mav",
    } ->
    file { "/srv/maverick/.c9":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    } ->
    file { "/srv/maverick/.c9/user.settings":
        ensure      => present,
        content     => template("maverick_web/user.settings.erb"),
        mode        => "644",
        owner       => "mav",
        group       => "mav",
        replace     => false,
    } ->
    file { "/srv/maverick/.c9/state.settings":
        ensure      => present,
        content     => template("maverick_web/state.settings.erb"),
        mode        => "644",
        owner       => "mav",
        group       => "mav",
        replace     => false,
    } ->
    file { "/etc/systemd/system/maverick-cloud9.service":
        content     => template("maverick_web/cloud9.service.erb"),
        owner       => "root",
        group       => "root",
        mode        => "644",
        notify      => Exec["maverick-systemctl-daemon-reload"],
    } ->
    service { "maverick-cloud9":
        ensure      => $_ensure,
        enable      => $_enable,
    }
    
    if defined(Class["::maverick_security"]) {
        maverick_security::firewall::firerule { "cloud9":
            ports       => $webport,
            ips         => lookup("firewall_ips"),
            proto       => "tcp"
        }
    }
    
}
