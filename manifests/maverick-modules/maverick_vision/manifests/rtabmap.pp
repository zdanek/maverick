class maverick_vision::rtabmap (
    $source = "https://github.com/introlab/rtabmap.git",
    $revision = "master",
) {
    
    if $raspberry_present == "yes" {
        ensure_packages(["libsqlite3-dev", "libpcl-dev", "cmake", "libfreenect-dev", "libopenni2-dev", "libqt4-dev", "libvtk5-qt4-dev"])
    } else {
        ensure_packages(["libsqlite3-dev", "libpcl-dev", "cmake", "libfreenect-dev", "libopenni2-dev", "libproj-dev", "libqt5svg5-dev"])
    }

    if ! ("install_flag_rtabmap" in $installflags) {
        oncevcsrepo { "git-rtabmap":
            gitsource   => $source,
            dest        => "/srv/maverick/var/build/rtabmap",
        } ->
        file { "/srv/maverick/var/build/rtabmap/build":
            ensure      => directory,
            owner       => "mav",
            group       => "mav",
            mode        => "755",
        } ->
        exec { "compile-rtabmap":
            user        => "mav",
            timeout     => 0,
            environment => ["PATH=/srv/maverick/software/opencv/bin:/srv/maverick/software/pangolin/bin:/srv/maverick/software/ros/current/bin:/usr/bin:/usr/sbin:/bin:/sbin:/usr/local/sbin", "CMAKE_PREFIX_PATH=/srv/maverick/software/opencv:/srv/maverick/software/librealsense"],
            cwd         => "/srv/maverick/var/build/rtabmap/build",
            command     => "/usr/bin/cmake -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/rtabmap -DCMAKE_INSTALL_RPATH=/srv/maverick/software/rtabmap/lib:/srv/maverick/software/librealsense/lib .. >/srv/maverick/var/log/build/rtabmap.cmake.log 2>&1 && make -j2 >/srv/maverick/var/log/build/rtabmap.make.log 2>&1 && make install >/srv/maverick/var/log/build/rtabmap.install.log 2>&1",
            creates     => "/srv/maverick/software/rtabmap/bin/rtabmap",
        } ->
        file { "/srv/maverick/var/build/.install_flag_rtabmap":
            owner       => "mav",
            group       => "mav",
            mode        => "644",
            ensure      => present,
        }
    }
    
}