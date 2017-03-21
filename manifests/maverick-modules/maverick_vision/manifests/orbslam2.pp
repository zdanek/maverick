class maverick_vision::orbslam2 (
) {
    
    ensure_packages(["libeigen3-dev", "libglew-dev", "libopenni-dev"])
    
    # Clone and Build pangolin
    oncevcsrepo { "git-pangolin":
        gitsource   => "https://github.com/stevenlovegrove/Pangolin.git",
        dest        => "/srv/maverick/var/build/pangolin",
    } ->
    # Create build directory
    file { "/srv/maverick/var/build/pangolin/build":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => 755,
    } ->
    exec { "pangolin-prepbuild":
        user        => "mav",
        timeout     => 0,
        environment => ["CMAKE_PREFIX_PATH=/srv/maverick/var/build/pangolin:/srv/maverick/var/build/pangolin/src", "CMAKE_MODULE_PATH=/srv/maverick/var/build/pangolin:/srv/maverick/var/build/pangolin/src", "Pangolin_DIR=/srv/maverick/var/build/pangolin"],
        command     => "/usr/bin/cmake -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/pangolin -DCMAKE_INSTALL_RPATH=/srv/maverick/software/pangolin/lib ..",
        cwd         => "/srv/maverick/var/build/pangolin/build",
        creates     => "/srv/maverick/var/build/pangolin/build/Makefile",
        require     => [ File["/srv/maverick/var/build/pangolin/build"], Package["libglew-dev"] ], # ensure we have all the dependencies satisfied
    } ->
    exec { "pangolin-build":
        user        => "mav",
        timeout     => 0,
        command     => "/usr/bin/make -j${::processorcount} >/srv/maverick/var/log/build/pangolin.build.out 2>&1",
        cwd         => "/srv/maverick/var/build/pangolin/build",
        creates     => "/srv/maverick/var/build/pangolin/build/src/libpangolin.so",
        require     => Exec["pangolin-prepbuild"],
    } ->
    exec { "pangolin-install":
        user        => "mav",
        timeout     => 0,
        command     => "/usr/bin/make install >/srv/maverick/var/log/build/pangolin.install.out 2>&1",
        cwd         => "/srv/maverick/var/build/pangolin/build",
        creates     => "/srv/maverick/software/pangolin/lib/libpangolin.so",
    }
    
    # Pull orb-slam2 from git mirror
    oncevcsrepo { "git-orbslam2":
        gitsource   => "https://github.com/raulmur/ORB_SLAM2.git",
        dest        => "/srv/maverick/var/build/orbslam2",
    } ->
    exec { "fix-orbslam2-build.sh":
        user        => "mav",
        command     => "/bin/sed -i -e 's/^make -j$/make -j 2/' build.sh",
        cwd         => "/srv/maverick/var/build/orbslam2",
        onlyif      => "/bin/grep -e '^make -j$' build.sh",
    } ->
    exec { "compile-orbslam2":
        user        => "mav",
        timeout     => 0,
        environment => ["LD_LIBRARY_PATH=/srv/maverick/software/opencv/lib:/srv/maverick/software/pangolin/lib", "PATH=/srv/maverick/software/opencv/bin:/srv/maverick/software/pangolin/bin:/usr/bin:/usr/sbin:/bin:/sbin:/usr/local/sbin", "CMAKE_PREFIX_PATH=/srv/maverick/software/opencv:/srv/maverick/software/pangolin"],
        cwd         => "/srv/maverick/var/build/orbslam2",
        command     => "/srv/maverick/var/build/orbslam2/build.sh",
        creates     => "/srv/maverick/var/build/orbslam2/lib/libORB_SLAM2.so",
        require     => Exec["pangolin-install"],
    } ->
    file { "/srv/maverick/software/orb_slam2":
        owner       => mav,
        group       => mav,
        mode        => "755",
        ensure      => directory,
    } ->
    exec { "install-orbslam2":
        user        => "mav",
        cwd         => "/srv/maverick/var/build/orbslam2",
        command     => "/bin/cp -R lib include Vocabulary Examples /srv/maverick/software/orb_slam2",
        creates     => "/srv/maverick/software/orb_slam2/lib/libORB_SLAM2.so",
    }
    
}