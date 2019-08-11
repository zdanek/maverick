class maverick_hardware::tegra (
    $jtx1inst = false,
) {

    ### Disable nvidia report_ip_to_host
    exec { "tegra-disable-report":
        command         => "/bin/mv -f /etc/rc.local /etc/rc.local.report",
        onlyif          => "/bin/grep report_ip_to_host /etc/rc.local",
    }
    
    if ! ("install_flag_jtx1inst" in $installflags) and $jtx1inst == true {
        # Pull jtx1inst fix from git mirror
        oncevcsrepo { "git-jtx1inst":
            gitsource   => "https://github.com/fnoop/jtx1inst.git",
            dest        => "/srv/maverick/var/build/jtx1inst",
            depth       => undef,
        } ->
        # Create build directory
        file { "/srv/maverick/var/build/jtx1inst/build":
            ensure      => directory,
            owner       => "mav",
            group       => "mav",
            mode        => "755",
        } ->
        exec { "jtx1inst-prepbuild":
            user        => "mav",
            timeout     => 0,
            environment => ["CMAKE_INSTALL_RPATH=/srv/maverick/software/jtx1inst/lib"],
            command     => "/usr/bin/cmake -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/jtx1inst -DCMAKE_INSTALL_RPATH=/srv/maverick/software/jtx1inst/lib .. >/srv/maverick/var/log/build/jtx1inst.build.out 2>&1",
            cwd         => "/srv/maverick/var/build/jtx1inst/build",
            creates     => "/srv/maverick/var/build/jtx1inst/build/Makefile",
        } ->
        exec { "jtx1inst-build":
            user        => "mav",
            timeout     => 0,
            command     => "/usr/bin/make >>/srv/maverick/var/log/build/jtx1inst.build.out 2>&1",
            cwd         => "/srv/maverick/var/build/jtx1inst/build",
            creates     => "/srv/maverick/var/build/jtx1inst/build/libjtx1inst.so",
            require     => Exec["jtx1inst-prepbuild"],
        } ->
        exec { "jtx1inst-install":
            user        => "mav",
            timeout     => 0,
            command     => "/usr/bin/make install >>/srv/maverick/var/log/build/jtx1inst.build.out 2>&1",
            cwd         => "/srv/maverick/var/build/jtx1inst/build",
            creates     => "/srv/maverick/software/jtx1inst/lib/libjtx1inst.so",
        } ->
        file { "/srv/maverick/software/jtx1inst/bin":
            ensure      => directory,
            mode        => "755",
            owner       => "mav",
            group       => "mav",
        } ->
        exec { "cp-jtx1inst-bin":
            command     => "/bin/cp /srv/maverick/var/build/jtx1inst/build/jtx1inst_demo /srv/maverick/software/jtx1inst/bin/jtx1inst",
            creates     => "/srv/maverick/software/jtx1inst/bin/jtx1inst",
            user        => "mav",
        } ->
        file { "/srv/maverick/var/build/.install_flag_jtx1inst":
            ensure      => present,
            owner       => "mav",
        }
    }

}
