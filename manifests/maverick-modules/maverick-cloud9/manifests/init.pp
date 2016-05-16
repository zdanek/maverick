class maverick-cloud9 (
    $enabled = true,
    $webport = "6789",
    $basepath = "/srv/maverick",
) {
    if $cloud9_installed == "no" {
        warning("Cloud9 will be compiled and can take a long time, please be patient..")
    }
    
    # Install system ncurses first, so cloud9 doesn't have to compile it
    ensure_packages(["libncurses5", "libncurses5-dev", "tmux"])

    oncevcsrepo { "git-cloud9":
        gitsource   => "https://github.com/c9/core.git",
        dest        => "/srv/maverick/software/cloud9",
        notify		=> Exec["install-cloud9"],
    } ->
    file { "/srv/maverick/software/cloud9/scripts/install.sh":
        ensure      => present,
        content     => template("maverick-cloud9/install.sh.erb"),
        mode        => 755,
        owner       => "mav",
        group       => "mav",
        require     => Oncevcsrepo["git-cloud9"]
    } ->
    exec { "install-preinstall-fix":
        # This is a fix for compiling dependencies on arm platforms
        creates     => "/srv/maverick/.c9/installed",
        command     => "/srv/maverick/software/cloud9/scripts/install.sh >/srv/maverick/data/logs/build/cloud9-deps.build.log 2>&1",
        timeout     => 0,
        user        => "mav",
        environment => ["HOME=/srv/maverick"],
    }
    exec { "install-cloud9":
        command		=> "/srv/maverick/software/cloud9/scripts/install-sdk.sh >/srv/maverick/data/logs/build/cloud9-sdk.build.log 2>&1",
        cwd		    => "/srv/maverick/software/cloud9",
        creates		=> "/srv/maverick/software/cloud9/node_modules/.gitignore",
        timeout		=> 0,
        user        => "mav",
        environment => ["HOME=/srv/maverick"],
    } ->
    file { "/etc/systemd/system/cloud9.service":
        content     => template("maverick-cloud9/cloud9.service.erb"),
        owner       => "root",
        group       => "root",
        mode        => 644,
        notify      => Exec["maverick-systemctl-daemon-reload"],
    } ->
    service { "cloud9":
        ensure      => running,
        enable      => true
    }
    
    if defined(Class["::maverick-security"]) {
        maverick-security::firewall::firerule { "cloud9":
            ports       => $webport,
            ips         => hiera("all_ips"),
            proto       => "tcp"
        }
    }
    
    exec { "change-cloud9-theme":
        command     => "/bin/sed /srv/maverick/.c9/user.settings -i -r -e 's/\"@theme\": \".*\"/\"@theme\": \"ace\\/theme\\/cloud9_day\"/'",
        unless      => "/bin/grep '@theme' /srv/maverick/.c9/user.settings | /bin/grep 'ace/theme/cloud9_day'",
        onlyif      => "/bin/ls /srv/maverick/.c9/user.settings",
    }
    
}
