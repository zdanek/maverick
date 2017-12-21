class maverick_intelligence::openkai (
) {

    /* This doesn't work yet */
    if ! ("install_flag_openkai" in $installflags) {

        # Pull openkai from git mirror
        oncevcsrepo { "git-openkai":
            gitsource   => "https://github.com/yankailab/OpenKAI.git",
            dest        => "/srv/maverick/var/build/openkai",
        } ->
        # Create build directory
        file { "/srv/maverick/var/build/openkai/build":
            ensure      => directory,
            owner       => "mav",
            group       => "mav",
            mode        => "755",
        } ->
        exec { "openkai-prepbuild":
            user        => "mav",
            timeout     => 0,
            environment => ["CPPFLAGS=-I/srv/maverick/software/opencv/include", "LD_LIBRARY_PATH=/srv/maverick/software/opencv/lib", "PATH=/srv/maverick/software/opencv/bin:/usr/bin:/usr/sbin:/bin:/sbin:/usr/local/sbin", "CMAKE_PREFIX_PATH=/srv/maverick/software/opencv", "CMAKE_INSTALL_RPATH=/srv/maverick/software/openkai/lib:/srv/maverick/software/opencv/lib"],
            command     => "/usr/bin/cmake -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/openkai -DCMAKE_INSTALL_RPATH=/srv/maverick/software/openkai/lib:/srv/maverick/software/opencv/lib ..",
            cwd         => "/srv/maverick/var/build/openkai/build",
            creates     => "/srv/maverick/var/build/openkai/build/Makefile",
            require     => [ Class["maverick_vision::opencv"], File["/srv/maverick/var/build/openkai/build"], File["/srv/maverick/var/build/.install_flag_opencv"] ], # ensure we have all the dependencies satisfied
        } ->
        exec { "openkai-build":
            user        => "mav",
            timeout     => 0,
            environment => ["CPPFLAGS=-I/srv/maverick/software/opencv/include"],
            command     => "/usr/bin/make -j${::processorcount} >/srv/maverick/var/log/build/openkai.build.out 2>&1",
            cwd         => "/srv/maverick/var/build/openkai/build",
            #creates     => "/srv/maverick/var/build/openkai/build/utils/openkai_tracker",
            require     => Exec["openkai-prepbuild"],
        } ->
        exec { "openkai-install":
            user        => "mav",
            timeout     => 0,
            command     => "/usr/bin/make install >/srv/maverick/var/log/build/openkai.install.out 2>&1",
            cwd         => "/srv/maverick/var/build/openkai/build",
            #creates     => "/srv/maverick/software/openkai/bin/openkai_tracker",
        } ->
        file { "/srv/maverick/var/build/.install_flag_openkai":
            ensure      => present,
            owner       => "mav",
        }

    }
    
}