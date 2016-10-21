class maverick_ros::ros (
    $installtype = "",
    $ros_distribution = "kinetic",
) {
    
    # If installtype is set then use it and skip autodetection
    if $installtype == "native" {
        $_installtype = "native"
    } elsif $installtype == "source" {
        $_installtype = "source"
    # First try and determine build type based on OS and architecture
    } elsif ($ros_distribution == "kinetic") {
    
        if (    ($operatingsystem == "Ubuntu" and $lsbdistcodename == "xenial" and ($architecture == "arm7l" or $architecture == "amd64" or $architecture == "i386")) or
                ($operatingsystem == "Ubuntu" and $lsbdistcodename == "wily" and ($architecture == "amd64" or $architecture == "i386")) or
                ($operatingsystem == "Debian" and $lsbdistcodename == "jessie" and ($architecture == "amd64" or $architecture == "arm64"))
        ) {
            $_installtype = "native"
        } else {
            $_installtype = "source"
        }
    } elsif $ros_distribution == "jade" {
        if (    ($operatingsystem == "Ubuntu" and $lsbdistcodename == "trusty" and $architecture == "arm7l") or
                ($operatingsystem == "Ubuntu" and ($lsbdistcodename =="trusty" or $lsbdistcodename == "utopic" or $lsbdistcodename == "vivid") and ($architecture == "amd64" or $architecture == "i386"))
        ) {
            $_installtype = "native"
        } else {
            $_installtype = "source"
        }
    }
    
    warning("ROS installtype: ${_installtype}")
    
    # Install ROS bootstrap from ros.org packages
    exec { "ros-repo":
        command     => '/bin/echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list',
        creates      => "/etc/apt/sources.list.d/ros-latest.list",
    } ->
    exec { "ros-repo-key":
        command     => "/usr/bin/wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | apt-key add -",
        unless      => "/usr/bin/apt-key list |/bin/grep ros.org",
    } ->
    exec { "ros-aptupdate":
        command     => "/usr/bin/apt-get update",
        unless      => "/usr/bin/dpkg -l python-rosinstall"
    } ->
    package { ["python-rosdep", "python-rosinstall", "python-rosinstall-generator"]:
        ensure      => installed,
        require     => Exec["ros-aptupdate"],
    }
    ensure_packages(["python-wstool", "build-essential"])

    # Install from ros repos
    if $installtype == "native" {
        $_package = "ros-${ros_distribution}-ros-base"
        package { ["${_package}"]:
            ensure      => installed
        }
        
        file { "/etc/profile.d/ros-env.sh":
            ensure      => present,
            mode        => 644,
            owner       => "root",
            group       => "root",
            content     => "source /opt/ros/${ros_distribution}/setup.bash",
        }
        
    # Build from source
    } elsif $installtype == "source" {
        file { "/srv/maverick/software/ros":
            ensure      => directory,
            owner       => "mav",
            group       => "mav",
            mode        => 755,
        }
        # Initialize rosdep
        exec { "rosdep-init":
            command         => "/usr/bin/rosdep init",
            creates         => "/etc/ros/rosdep/sources.list.d/20-default.list",
            require         => Package["python-rosdep"]
        } ->
        exec { "rosdep-update":
            user            => "mav",
            command         => "/usr/bin/rosdep update",
            creates         => "/srv/maverick/.ros/rosdep/sources.cache",
            require         => Package["python-rosdep"]
        } ->
        file { "/srv/maverick/var/build/ros_catkin_ws":
            ensure          => directory,
            owner           => "mav",
            group           => "mav",
        } ->
        exec { "catkin_rosinstall":
            command         => "/usr/bin/rosinstall_generator ros_comm --rosdistro jade --deps --wet-only --tar > jade-ros_comm-wet.rosinstall && /usr/bin/wstool init -j${::processorcount} src jade-ros_comm-wet.rosinstall",
            cwd             => "/srv/maverick/var/build/ros_catkin_ws",
            user            => "mav",
            creates         => "/srv/maverick/var/build/ros_catkin_ws/src/.rosinstall"
        } ->
        exec { "rosdep-install":
            command         => "/usr/bin/rosdep install --from-paths src --ignore-src --rosdistro jade -y",
            cwd             => "/srv/maverick/var/build/ros_catkin_ws",
            unless          => "/usr/bin/dpkg -l libboost-all-dev"
        } ->
        exec { "catkin_make":
            command         => "/srv/maverick/var/build/ros_catkin_ws/src/catkin/bin/catkin_make_isolated --install --install-space /srv/maverick/software/ros -DCMAKE_BUILD_TYPE=Release -j${::processorcount}",
            cwd             => "/srv/maverick/var/build/ros_catkin_ws",
            user            => "mav",
            creates         => "/srv/maverick/software/ros/lib/rosbag/topic_renamer.py",
            timeout         => 0,
            require         => File["/srv/maverick/software/ros"]
        }
        
        # Add opencv to the existing workspace through vision_opencv package, this also installs std_msgs package as dependency
        ensure_packages(["libpoco-dev", "libyaml-cpp-dev"])
        exec { "ws_add_opencv":
            command         => "/usr/bin/rosinstall_generator vision_opencv --rosdistro jade --deps --wet-only --tar >jade-vision_opencv-wet.rosinstall && /usr/bin/wstool merge -t src jade-vision_opencv-wet.rosinstall && /usr/bin/wstool update -t src",
            cwd             => "/srv/maverick/var/build/ros_catkin_ws",
            user            => "mav",
            creates         => "/srv/maverick/var/build/ros_catkin_ws/src/vision_opencv",
            require         => Package["libpoco-dev"]
        } ->
        exec { "catkin_make_vision_opencv":
            command         => "/srv/maverick/var/build/ros_catkin_ws/src/catkin/bin/catkin_make_isolated --install --install-space /srv/maverick/software/ros -DCMAKE_BUILD_TYPE=Release -j${::processorcount}",
            cwd             => "/srv/maverick/var/build/ros_catkin_ws",
            user            => "mav",
            creates         => "/srv/maverick/software/ros/share/opencv_apps/nodelet_plugins.xml",
            timeout         => 0,
            require         => File["/srv/maverick/software/ros"]
        }

        # Add mavros to the existing workspace, this also installs mavlink package as dependency
        exec { "ws_add_mavros":
            command         => "/usr/bin/rosinstall_generator mavros --rosdistro jade --deps --wet-only --tar >jade-mavros-wet.rosinstall && /usr/bin/rosinstall_generator visualization_msgs --rosdistro jade --deps --wet-only --tar >>jade-mavros-wet.rosinstall && /usr/bin/rosinstall_generator urdf --rosdistro jade --deps --wet-only --tar >>jade-mavros-wet.rosinstall && /usr/bin/wstool merge -t src jade-mavros-wet.rosinstall && /usr/bin/wstool update -t src",
            cwd             => "/srv/maverick/var/build/ros_catkin_ws",
            user            => "mav",
            creates         => "/srv/maverick/var/build/ros_catkin_ws/src/mavros",
        }
        exec { "catkin_make_mavros":
            # Note must only use -j1 otherwise we get compiler errors
            command         => "/srv/maverick/var/build/ros_catkin_ws/src/catkin/bin/catkin_make_isolated --install --install-space /srv/maverick/software/ros -DCMAKE_BUILD_TYPE=Release -j1",
            cwd             => "/srv/maverick/var/build/ros_catkin_ws",
            user            => "mav",
            creates         => "/srv/maverick/software/ros/lib/libmavros.so",
            timeout         => 0,
            require         => File["/srv/maverick/software/ros"]
        }

        file { "/etc/profile.d/ros-env.sh":
            ensure      => present,
            mode        => 644,
            owner       => "root",
            group       => "root",
            content     => "source ~/software/ros/setup.bash",
        }
    
    }  
    
}