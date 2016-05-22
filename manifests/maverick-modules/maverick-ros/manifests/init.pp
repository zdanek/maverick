class maverick-ros (
    $ros_installtype = "native"
) {

    file { "/srv/maverick/software/ros":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => 755,
    }
    
    # Install ROS from ros.org packages
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
    if $ros_installtype == "native" {
        package { ["ros-base-ros-jade"]:
            ensure      => installed
        }
    
    # Build from source
    } elsif $ros_installtype == "source" {
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
        file { "/srv/maverick/build/ros_catkin_ws":
            ensure          => directory,
            owner           => "mav",
            group           => "mav",
        } ->
        exec { "catkin_rosinstall":
            command         => "/usr/bin/rosinstall_generator ros_comm --rosdistro jade --deps --wet-only --tar > jade-ros_comm-wet.rosinstall && /usr/bin/wstool init -j${::processorcount} src jade-ros_comm-wet.rosinstall",
            cwd             => "/srv/maverick/build/ros_catkin_ws",
            user            => "mav",
            creates         => "/srv/maverick/build/ros_catkin_ws/src/.rosinstall"
        } ->
        exec { "rosdep-install":
            command         => "/usr/bin/rosdep install --from-paths src --ignore-src --rosdistro jade -y",
            cwd             => "/srv/maverick/build/ros_catkin_ws",
            unless          => "/usr/bin/dpkg -l libboost-all-dev"
        } ->
        exec { "catkin_make":
            command         => "/srv/maverick/build/ros_catkin_ws/src/catkin/bin/catkin_make_isolated --install --install-space /srv/maverick/software/ros -DCMAKE_BUILD_TYPE=Release -j${::processorcount}",
            cwd             => "/srv/maverick/build/ros_catkin_ws",
            user            => "mav",
            creates         => "/srv/maverick/software/ros/lib/rosbag/topic_renamer.py",
            timeout         => 0,
            require         => File["/srv/maverick/software/ros"]
        }
        
        # Add opencv to the existing workspace through vision_opencv package, this also installs std_msgs package as dependency
        ensure_packages(["libpoco-dev", "libyaml-cpp-dev "])
        exec { "ws_add_opencv":
            command         => "/usr/bin/rosinstall_generator vision_opencv --rosdistro jade --deps --wet-only --tar >jade-vision_opencv-wet.rosinstall && /usr/bin/wstool merge -t src jade-vision_opencv-wet.rosinstall && /usr/bin/wstool update -t src",
            cwd             => "/srv/maverick/build/ros_catkin_ws",
            user            => "mav",
            creates         => "/srv/maverick/build/ros_catkin_ws/src/vision_opencv",
            require         => Package["libpoco-dev"]
        } ->
        exec { "catkin_make_vision_opencv":
            command         => "/srv/maverick/build/ros_catkin_ws/src/catkin/bin/catkin_make_isolated --install --install-space /srv/maverick/software/ros -DCMAKE_BUILD_TYPE=Release -j${::processorcount}",
            cwd             => "/srv/maverick/build/ros_catkin_ws",
            user            => "mav",
            creates         => "/srv/maverick/software/ros/share/opencv_apps/nodelet_plugins.xml",
            timeout         => 0,
            require         => File["/srv/maverick/software/ros"]
        }

    }   

    file { "/etc/profile.d/ros-env.sh":
        ensure      => present,
        mode        => 644,
        owner       => "root",
        group       => "root",
        content     => "source ~/software/ros/setup.bash",
    }
    
}