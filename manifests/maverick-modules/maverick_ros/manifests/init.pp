class maverick_ros (
    $installtype = "source",
    $distribution = "kinetic",
    $installdir = "/srv/maverick/software/ros",
    $mavros_sitl_active = true,
    $mavros_fc_active = true,
) {
    
    # If installtype is set then use it and skip autodetection
    if $installtype == "native" {
        if $ros_installed == "no" { 
            notice("ROS: Native installation requested")
        }
        $_installtype = "native"
    } elsif $installtype == "source" {
        if $ros_installed == "no" {
            notice("ROS: Source installation requested")
        }
        $_installtype = "source"
    # First try and determine build type based on OS and architecture
    } else {
        if ($distribution == "kinetic") {
            if (    ($operatingsystem == "Ubuntu" and $lsbdistcodename == "xenial" and ($architecture == "armv7l" or $architecture == "amd64" or $architecture == "i386")) or
                    ($operatingsystem == "Ubuntu" and $lsbdistcodename == "wily" and ($architecture == "amd64" or $architecture == "i386")) or
                    ($operatingsystem == "Debian" and $lsbdistcodename == "jessie" and ($architecture == "amd64" or $architecture == "arm64"))
            ) {
                $_installtype = "native"
            } else {
                $_installtype = "source"
            }
        } elsif $distribution == "jade" {
            if (    ($operatingsystem == "Ubuntu" and $lsbdistcodename == "trusty" and $architecture == "armv7l") or
                    ($operatingsystem == "Ubuntu" and ($lsbdistcodename =="trusty" or $lsbdistcodename == "utopic" or $lsbdistcodename == "vivid") and ($architecture == "amd64" or $architecture == "i386"))
            ) {
                $_installtype = "native"
            } else {
                $_installtype = "source"
            }
        }
    
        if $_installtype == "native" and $ros_installed == "no" {
            notice("ROS: supported platform detected for ${distribution} distribution, using native packages")
        } elsif $_installtype == "source" and $ros_installed == "no" {
            notice("ROS: unsupported platform for ${distribution} distribution, installing from source")
        }
    }
    
    # Install ROS bootstrap from ros.org packages
    exec { "ros-repo":
        command     => '/bin/echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list',
        creates     => "/etc/apt/sources.list.d/ros-latest.list",
        require     => Class["maverick_vision::opencv"],
    } ->
    exec { "ros-repo-key":
        #command     => "/usr/bin/wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | apt-key add -",
        command     => "/usr/bin/apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116",
        unless      => "/usr/bin/apt-key list |/bin/grep B01FA116",
    } ->
    exec { "ros-aptupdate":
        command     => "/usr/bin/apt-get update",
        unless      => "/usr/bin/dpkg -l python-rosinstall"
    } ->
    package { ["python-rosdep", "python-rosinstall", "python-rosinstall-generator"]:
        ensure      => installed,
        require     => Exec["ros-aptupdate"],
    }
    ensure_packages(["build-essential"])
    $wstool_package = $::operatingsystem ? {
        'Ubuntu'        => 'python-wstool',
        'Debian'        => 'python-wstools',
    }
    package { "python-wstool":
        name        => $wstool_package
    }

    # Install from ros repos
    if $_installtype == "native" {
        package { ["ros-${distribution}-ros-base", "ros-${distribution}-mavros", "ros-${distribution}-mavros-extras", "ros-${distribution}-mavros-msgs", "ros-${distribution}-test-mavros", "ros-${distribution}-vision-opencv"]:
            ensure      => installed,
            require     => [ Exec["ros-aptupdate"], Package["python-rosdep"] ],
        } ->
        file { "/etc/profile.d/30-ros-env.sh":
            ensure      => present,
            mode        => 644,
            owner       => "root",
            group       => "root",
            content     => "source /opt/ros/${distribution}/setup.bash",
        }
        
    # Build from source
    } elsif $_installtype == "source" {
        file { ["${installdir}", "${installdir}/${distribution}"]:
            ensure      => directory,
            owner       => "mav",
            group       => "mav",
            mode        => 755,
        }
        $_builddir = "/srv/maverick/var/build/ros_catkin_ws"
        file { "${_builddir}":
            ensure      => directory,
            owner       => "mav",
            group       => "mav",
            mode        => 755,
        }
        $buildparallel = ceiling((0 + $::processorcount) / 2) # Restrict build parallelization to roughly processors/2
        
        # Initialize rosdep
        exec { "rosdep-init":
            command         => "/usr/bin/rosdep init",
            creates         => "/etc/ros/rosdep/sources.list.d/20-default.list",
            require         => [ Package["python-rosdep"], Package["python-wstool"] ]
        } ->
        exec { "rosdep-update":
            user            => "mav",
            command         => "/usr/bin/rosdep update",
            creates         => "/srv/maverick/.ros/rosdep/sources.cache",
            require         => Package["python-rosdep"]
        } ->
        exec { "catkin_rosinstall":
            command         => "/usr/bin/rosinstall_generator ros_comm --rosdistro ${distribution} --deps --wet-only --tar > ${distribution}-ros_comm-wet.rosinstall && /usr/bin/wstool init -j${buildparallel} src ${distribution}-ros_comm-wet.rosinstall",
            cwd             => "${_builddir}",
            user            => "mav",
            creates         => "${_builddir}/src/.rosinstall"
        } ->
        exec { "rosdep-install":
            command         => "/usr/bin/rosdep install --from-paths src --ignore-src --rosdistro ${distribution} -y",
            cwd             => "${_builddir}",
            user            => "mav",
            timeout         => 0,
            unless          => "/usr/bin/rosdep check --from-paths src --ignore-src --rosdistro ${distribution} -y |/bin/grep 'have been satis'",
        } ->
        exec { "catkin_make":
            command         => "${_builddir}/src/catkin/bin/catkin_make_isolated --install --install-space ${installdir}/${distribution} -DCMAKE_BUILD_TYPE=Release -j${buildparallel}",
            cwd             => "${_builddir}",
            user            => "mav",
            creates         => "${installdir}/${distribution}/lib/rosbag/topic_renamer.py",
            timeout         => 0,
            require         => File["${installdir}/${distribution}"]
        }
        
        # Add opencv to the existing workspace through vision_opencv package
        ensure_packages(["libpoco-dev", "libyaml-cpp-dev"])
        exec { "ws_add_opencv":
            command         => "/usr/bin/rosinstall_generator vision_opencv --rosdistro ${distribution} --deps --wet-only --tar >${distribution}-vision_opencv-wet.rosinstall && /usr/bin/wstool merge -t src ${distribution}-vision_opencv-wet.rosinstall && /usr/bin/wstool update -t src",
            cwd             => "${_builddir}",
            user            => "mav",
            creates         => "${_builddir}/src/vision_opencv",
            timeout         => 0,
            require         => [ Package["libpoco-dev"], Exec["catkin_make"] ]
        } ->
        exec { "catkin_make_vision_opencv":
            command         => "${_builddir}/src/catkin/bin/catkin_make_isolated --install --install-space ${installdir}/${distribution} -DCMAKE_BUILD_TYPE=Release -j${buildparallel}",
            cwd             => "${_builddir}",
            user            => "mav",
            creates         => "${installdir}/${distribution}/lib/libopencv_optflow3.so.3.1.0",
            timeout         => 0,
            require         => File["${installdir}/${distribution}"]
        }

        # Add mavros to the existing workspace, this also installs mavlink package as dependency
        exec { "ws_add_mavros":
            command         => "/usr/bin/rosinstall_generator mavros --rosdistro ${distribution} --deps --wet-only --tar >${distribution}-mavros-wet.rosinstall && /usr/bin/rosinstall_generator visualization_msgs --rosdistro ${distribution} --deps --wet-only --tar >>${distribution}-mavros-wet.rosinstall && /usr/bin/rosinstall_generator urdf --rosdistro ${distribution} --deps --wet-only --tar >>${distribution}-mavros-wet.rosinstall && /usr/bin/wstool merge -t src ${distribution}-mavros-wet.rosinstall && /usr/bin/wstool update -t src",
            cwd             => "${_builddir}",
            user            => "mav",
            creates         => "${_builddir}/src/mavros",
            timeout         => 0,
            require         => Exec["catkin_make"]
        } ->
        exec { "catkin_make_mavros":
            # Note must only use -j1 otherwise we get compiler errors
            command         => "/usr/bin/rosdep install --from-paths src --ignore-src --rosdistro ${distribution} -y && ${_builddir}/src/catkin/bin/catkin_make_isolated --install --install-space ${installdir}/${distribution} -DCMAKE_BUILD_TYPE=Release -j1",
            cwd             => "${_builddir}",
            user            => "mav",
            creates         => "${installdir}/${distribution}/lib/libmavros.so",
            timeout         => 0,
            require         => File["${installdir}/${distribution}"]
        }

        # Create symlink to usual vendor install directory
        file { ["/opt", "/opt/ros"]:
            ensure      => directory,
            mode        => "755",
            owner       => "root",
            group       => "root",
        } ->
        file { "/opt/ros/${distribution}":
            ensure      => link,
            target      => "${installdir}/${distribution}",
            force       => true,
        } ->
        file { "/etc/profile.d/30-ros-env.sh":
            ensure      => present,
            mode        => 644,
            owner       => "root",
            group       => "root",
            content     => "source /opt/ros/${distribution}/setup.bash",
            require         => Exec["catkin_make"]
        }
    }  
    
    # If SITL is active, add a mavros service for sitl link
    if defined("maverick_dev::sitl") and $mavros_sitl_active == true {
        file { "/etc/systemd/system/maverick-mavros-sitl.service":
            content     => template("maverick_ros/maverick-mavros-sitl.service.erb"),
            owner       => "root",
            group       => "root",
            mode        => 644,
            notify      => Exec["maverick-systemctl-daemon-reload"],
        } ->
        service { "maverick-mavros-sitl":
            ensure      => running,
            enable      => true,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/profile.d/30-ros-env.sh"] ]
        }
    } else {
        service { "maverick-mavros-sitl":
            ensure      => stopped,
            enable      => false,
            require     => [ Exec["maverick-systemctl-daemon-reload"] ]
        }
    }
    
    # If FC is active, add a mavros service for FC link
    if defined("maverick_fc") and $mavros_fc_active == true {
        file { "/etc/systemd/system/maverick-mavros-fc.service":
            content     => template("maverick_ros/maverick-mavros-fc.service.erb"),
            owner       => "root",
            group       => "root",
            mode        => 644,
            notify      => Exec["maverick-systemctl-daemon-reload"],
        } ->
        service { "maverick-mavros-fc":
            ensure      => running,
            enable      => true,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/profile.d/30-ros-env.sh"] ]
        }
    } else {
        service { "maverick-mavros-fc":
            ensure      => stopped,
            enable      => false,
            require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/profile.d/30-ros-env.sh"] ]
        }

    }
    
}