class maverick_vision::aruco (
    $aruco_gitsource = "https://github.com/fnoop/aruco.git",
    $aruco_gitbranch = "2.0.20-git-fixed",
) {

    if ! ("install_flag_aruco" in $installflags) {

        # Pull aruco from git mirror
        oncevcsrepo { "git-aruco":
            gitsource   => $aruco_gitsource,
            revision    => $aruco_gitbranch,
            dest        => "/srv/maverick/var/build/aruco",
        } ->
        # Create build directory
        file { "/srv/maverick/var/build/aruco/build":
            ensure      => directory,
            owner       => "mav",
            group       => "mav",
            mode        => 755,
        }
        
        exec { "aruco-prepbuild":
            user        => "mav",
            timeout     => 0,
            environment => ["LD_LIBRARY_PATH=/srv/maverick/software/opencv/lib", "PATH=/srv/maverick/software/opencv/bin:/usr/bin:/usr/sbin:/bin:/sbin:/usr/local/sbin", "CMAKE_PREFIX_PATH=/srv/maverick/software/opencv"],
            command     => "/usr/bin/cmake -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/aruco -DCMAKE_INSTALL_RPATH=/srv/maverick/software/aruco/lib ..",
            cwd         => "/srv/maverick/var/build/aruco/build",
            creates     => "/srv/maverick/var/build/aruco/build/Makefile",
            require     => [ File["/srv/maverick/var/build/aruco/build"], File["/srv/maverick/var/build/.install_flag_opencv"] ], # ensure we have all the dependencies satisfied
        } ->
        exec { "aruco-build":
            user        => "mav",
            timeout     => 0,
            command     => "/usr/bin/make -j${::processorcount} >/srv/maverick/var/log/build/aruco.build.out 2>&1",
            cwd         => "/srv/maverick/var/build/aruco/build",
            creates     => "/srv/maverick/var/build/aruco/build/utils/aruco_tracker",
            require     => Exec["aruco-prepbuild"],
        } ->
        exec { "aruco-install":
            user        => "mav",
            timeout     => 0,
            command     => "/usr/bin/make install >/srv/maverick/var/log/build/aruco.install.out 2>&1",
            cwd         => "/srv/maverick/var/build/aruco/build",
            creates     => "/srv/maverick/software/aruco/bin/aruco_tracker",
        } ->
        file { "/etc/profile.d/60-maverick-aruco-path.sh":
            mode        => 644,
            owner       => "root",
            group       => "root",
            content     => "export PATH=/srv/maverick/software/aruco/bin:\$PATH",
        } ->
        file { "/etc/profile.d/60-maverick-aruco-pkgconfig.sh":
            mode        => 644,
            owner       => "root",
            group       => "root",
            content     => "export PKG_CONFIG_PATH=/srv/maverick/software/aruco/lib/pkgconfig:\$PKG_CONFIG_PATH",
        } ->
        file { "/etc/profile.d/40-maverick-aruco-ldlibrarypath.sh":
            mode        => 644,
            owner       => "root",
            group       => "root",
            content     => "export LD_LIBRARY_PATH=/srv/maverick/software/aruco/lib:\$LD_LIBRARY_PATH",
        } ->
        file { "/etc/profile.d/40-maverick-aruco-cmake.sh":
            mode        => 644,
            owner       => "root",
            group       => "root",
            content     => "export CMAKE_PREFIX_PATH=\$CMAKE_PREFIX_PATH:/srv/maverick/software/aruco",
        }

        file { "/srv/maverick/var/build/.install_flag_aruco":
            ensure      => present,
            owner       => "mav",
        }
       
    }
}