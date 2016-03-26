class maverick-cloud9 (
    $enabled = true,
    $webport = "6789",
    $basepath = "/srv/maverick",
) {
    if $cloud9_installed == "no" {
        warning("Cloud9 will be compiled and can take a long time, please be patient..")
    }
    file { "/srv/maverick/software/cloud9":
        ensure 		=> directory,
        require		=> File["/srv/maverick/software"],
        mode		=> 755,
        owner		=> "mav",
        group		=> "mav",
    } ->
    vcsrepo { "/srv/maverick/software/cloud9":
        ensure		=> present,
        provider 	=> git,
        source		=> "https://github.com/c9/core.git",
        revision	=> "master",
        owner		=> "mav",
        group		=> "mav",
        notify		=> [ Exec["install-cloud9"] ]
    } ->
    file { "/srv/maverick/software/cloud9/scripts/install.sh":
        ensure      => present,
        source      => "puppet:///modules/maverick-cloud9/install.sh",
        mode        => 755,
        owner       => "mav",
        group       => "mav",
    } ->
    exec { "install-preinstall-fix":
        # This is a fix for compiling dependencies on arm platforms
        creates     => "/srv/maverick/.c9/installed",
        command     => "/srv/maverick/software/cloud9/scripts/install.sh",
        timeout     => 0,
        user        => "mav",
        environment => ["HOME=/srv/maverick"],
    }
    exec { "install-cloud9":
        command		=> "/srv/maverick/software/cloud9/scripts/install-sdk.sh",
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
        mode        => 644
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
    
}
