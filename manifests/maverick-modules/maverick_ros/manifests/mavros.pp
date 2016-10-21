class maverick_ros::mavros (
) {
    
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
    
}