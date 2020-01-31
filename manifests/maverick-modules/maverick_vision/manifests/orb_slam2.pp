# @summary
#   Maverick_vision::Orb_slam2 class
#   This class installs and manages the Orb_slam2 software.
#
# @example Declaring the class
#   This class is included from maverick_vision class and should not be included from elsewhere
#
# @param ros_build
#   If true, add the orb_slam2 ROS components
#
class maverick_vision::orb_slam2 (
    Boolean $ros_build = false,
) {
    
    ensure_packages(["libeigen3-dev", "libglew-dev", "libopenni2-dev"])
    
    if ! ("install_flag_pangolin" in $installflags) {
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
            mode        => "755",
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
            before      => Exec["compile-orb_slam2"],
        } ->
        file { "/srv/maverick/var/build/.install_flag_pangolin":
            owner       => "mav",
            group       => "mav",
            mode        => "644",
            ensure      => present,
        }
    }
    
    # Pull orb-slam2 from git mirror
    oncevcsrepo { "git-orb_slam2":
        gitsource   => "https://github.com/raulmur/ORB_SLAM2.git",
        dest        => "/srv/maverick/software/orb_slam2",
    } ->
    exec { "fix-orb_slam2-build.sh":
        user        => "mav",
        command     => "/bin/sed -i -e 's/^make -j$/make -j 2/' build.sh build_ros.sh",
        cwd         => "/srv/maverick/software/orb_slam2",
        onlyif      => "/bin/grep -e '^make -j$' build.sh",
    } ->
    # Hack orb_slam2 code to actually build, https://github.com/raulmur/ORB_SLAM2/pull/144
    exec {"fix-orb_slam2-usleep":
        user        => "mav",
        command     => "/bin/sed -i -e 's/#include<string>/#include<string>\\n#include <unistd.h>/' /srv/maverick/software/orb_slam2/include/System.h",
        unless      => "/bin/grep -e 'unistd.h' /srv/maverick/software/orb_slam2/include/System.h",
    } ->
    file { ["/srv/maverick/software/orb_slam2/build", "/srv/maverick/software/orb_slam2/Examples/ROS/ORB_SLAM2/build", "/srv/maverick/software/orb_slam2/Thirdparty/DBoW2/build", "/srv/maverick/software/orb_slam2/Thirdparty/g2o/build"]:
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    } ->
    exec { "compile-orb_slam2_dbow2":
        user        => "mav",
        timeout     => 0,
        environment => ["CMAKE_PREFIX_PATH=/srv/maverick/software/opencv"],
        cwd         => "/srv/maverick/software/orb_slam2/Thirdparty/DBoW2/build",
        command     => "/usr/bin/cmake .. -DROS_BUILD_TYPE=Release && /usr/bin/make -j2 >/srv/maverick/var/log/build/orb_slam2.dbow2.log 2>&1",
        creates     => "/srv/maverick/software/orb_slam2/Thirdparty/DBoW2/lib/libDBoW2.so",
    } ->
    exec { "compile-orb_slam2_g2o":
        user        => "mav",
        timeout     => 0,
        cwd         => "/srv/maverick/software/orb_slam2/Thirdparty/g2o/build",
        command     => "/usr/bin/cmake .. -DROS_BUILD_TYPE=Release && /usr/bin/make -j2 >/srv/maverick/var/log/build/orb_slam2.g2o.log 2>&1",
        creates     => "/srv/maverick/software/orb_slam2/Thirdparty/g2o/lib/libg2o.so",
    } ->
    exec { "compile-orb_slam2":
        user        => "mav",
        timeout     => 0,
        environment => ["PATH=/srv/maverick/software/opencv/bin:/srv/maverick/software/pangolin/bin:/srv/maverick/software/ros/current/bin:/usr/bin:/usr/sbin:/bin:/sbin:/usr/local/sbin", "CMAKE_PREFIX_PATH=/srv/maverick/software/opencv:/srv/maverick/software/pangolin", "Pangolin_DIR=/srv/maverick/var/build/pangolin"],
        cwd         => "/srv/maverick/software/orb_slam2/build",
        command     => "/usr/bin/cmake .. -DROS_BUILD_TYPE=Release -DCMAKE_INSTALL_RPATH=/srv/maverick/software/opencv/lib -DCMAKE_MODULE_PATH=/srv/maverick/software/opencv && /usr/bin/make -j2 >/srv/maverick/var/log/build/orb_slam2.log 2>&1",
        creates     => "/srv/maverick/software/orb_slam2/Examples/Monocular/mono_euroc",
    }
    if $raspberry_present != "yes" and $ros_build == true {
        exec { "compile-orb_slam2_ros":
            user        => "mav",
            timeout     => 0,
            environment => ["PATH=/srv/maverick/software/opencv/bin:/srv/maverick/software/pangolin/bin:/srv/maverick/software/ros/current/bin:/usr/bin:/usr/sbin:/bin:/sbin:/usr/local/sbin", "CMAKE_PREFIX_PATH=/srv/maverick/software/opencv:/srv/maverick/software/pangolin", "Pangolin_DIR=/srv/maverick/var/build/pangolin"],
            cwd         => "/srv/maverick/software/orb_slam2/Examples/ROS/ORB_SLAM2/build",
            command     => "/bin/bash -c 'source /srv/maverick/software/ros/current/setup.bash && export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:/srv/maverick/software/orb_slam2/Examples/ROS/ORB_SLAM2 && /usr/bin/cmake .. -DROS_BUILD_TYPE=Release -DCMAKE_INSTALL_RPATH=/srv/maverick/software/opencv/lib -DCMAKE_MODULE_PATH=/srv/maverick/software/opencv && /usr/bin/make >/srv/maverick/var/log/build/orb_slam2.ros.log 2>&1'",
            creates     => "/srv/maverick/software/orb_slam2/Examples/ROS/ORB_SLAM2/Mono",
            require     => [ Exec["compile-orb_slam2"], Class["maverick_ros"] ]
        }
    }
    file { "/etc/profile.d/70-maverick-orb_slam2-ros_package_path.sh":
        content     => "ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:/srv/maverick/software/orb_slam2/Examples/ROS/ORB_SLAM2",
        owner       => "root",
        group       => "root",
        mode        => "644",
        require     => Exec["compile-orb_slam2"],
    } ->
    file { "/etc/ld.so.conf.d/maverick-orb_slam2.conf":
        mode        => "644",
        owner       => "root",
        group       => "root",
        content     => "/srv/maverick/software/orb_slam2/lib\n/srv/maverick/software/pangolin/lib",
        notify      => Exec["maverick-ldconfig"],
    }

}
