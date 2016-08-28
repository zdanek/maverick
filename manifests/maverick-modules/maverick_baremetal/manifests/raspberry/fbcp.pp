class maverick_baremetal::raspberry::fbcp (
) {
    
    ensure_packages(["libbsd-dev", "cmake"])
    oncevcsrepo { "git-raspi2fb":
        gitsource   => "https://github.com/AndrewFromMelbourne/raspi2fb.git",
        dest        => "/srv/maverick/build/raspi2fb",
    } ->
    file { "/srv/maverick/build/raspi2fb/build":
        ensure      => directory,
    } ->
    exec { "build-raspi2fb":
        timeout     => 0,
        command     => "/usr/bin/cmake .. && /usr/bin/make && /usr/bin/make install >/srv/maverick/var/log/build/raspi2fb.build.out 2>&1",
        cwd         => "/srv/maverick/build/raspi2fb/build",
        creates     => "/usr/local/bin/raspi2fb",
        require     => [ Package["cmake"], Oncevcsrepo["git-raspi2fb"], Package["libbsd-dev"] ] # ensure we have all the dependencies satisfied
    } ->
    exec { "install-raspi2fb-service":
        command     => "/bin/cp /srv/maverick/build/raspi2fb/raspi2fb@.service /etc/systemd/system",
        creates     => "/etc/systemd/system/raspi2fb@.service",
    } ->
    service { "raspi2fb@1":
        enable      => true,
        ensure      => running
    }
    
}