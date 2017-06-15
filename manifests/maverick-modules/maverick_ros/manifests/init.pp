class maverick_ros (
    $installtype = "",
    $distribution = "kinetic",
    $buildtype = "ros_comm", # ROS core variant, ros_comm is base without GUI, can also be desktop, desktop_full.  mobile and perception variants useful for drones.  desktop_full includes everything.
    $binarytype = ["ros-kinetic-perception", "ros-kinetic-viz"], # Binary packages install type, can be ros-kinetic-ros-base, ros-kinetic-desktop, ros-kinetic-desktop-full, ros-kinetic-viz, ros-kinetic-perception
    $builddir = "/srv/maverick/var/build/ros_catkin_ws",
    $installdir = "/srv/maverick/software/ros",
    $module_mavros = true,
    $module_opencv = false,
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
            if (    ($operatingsystem == "Ubuntu" and $lsbdistcodename == "xenial" and ($architecture == "armv7l" or $architecture == "armv6l" or $architecture == "amd64" or $architecture == "i386")) or
                    ($operatingsystem == "Ubuntu" and $lsbdistcodename == "wily" and ($architecture == "amd64" or $architecture == "i386")) or
                    ($operatingsystem == "Debian" and $lsbdistcodename == "jessie" and ($architecture == "amd64" or $architecture == "arm64"))
            ) {
                $_installtype = "native"
            } else {
                $_installtype = "source"
            }
        } elsif $distribution == "jade" {
            if (    ($operatingsystem == "Ubuntu" and $lsbdistcodename == "trusty" and ($architecture == "armv7l" or $architecture == "armv6l")) or
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
    
    # Create symlink to usual vendor install directory
    file { ["/opt", "/opt/ros"]:
        ensure      => directory,
        mode        => "755",
        owner       => "root",
        group       => "root",
    } ->
    file { ["${installdir}", "${installdir}/${distribution}"]:
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    } ->
    file { "/opt/ros/${distribution}":
        ensure      => link,
        target      => "${installdir}/${distribution}",
        force       => true,
    } ->
    file { "${installdir}/current":
        ensure      => link,
        target      => "${installdir}/${distribution}",
        force       => true,
    } ->
    # Install ROS bootstrap from ros.org packages
    exec { "ros-repo-key":
        command     => "/usr/bin/apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116",
        unless      => "/usr/bin/apt-key list |/bin/egrep 'B01F\s?A116'",
    } ->
    exec { "ros-repo":
        command     => '/bin/echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list',
        creates     => "/etc/apt/sources.list.d/ros-latest.list",
        notify      => Exec["maverick-aptget-update"],
    } ->
    exec { "ros-aptupdate":
        command     => "/usr/bin/apt-get update",
        unless      => "/bin/ls /var/lib/apt/lists/*ros.org*"
    } ->
    package { ["python-rosdep"]:
        ensure      => installed,
        require     => [Exec["maverick-aptget-update"], Exec["ros-aptupdate"]],
    }
    $wstool_package = $::operatingsystem ? {
        'Ubuntu'        => 'python-wstool',
        'Debian'        => 'python-wstools',
    }
    package { "python-wstool":
        name        => $wstool_package
    }

    # Install from ros repos
    if $_installtype == "native" {
        package { [$binarytype, "ros-${distribution}-mavros", "ros-${distribution}-mavros-extras", "ros-${distribution}-mavros-msgs", "ros-${distribution}-test-mavros"]:
            ensure      => installed,
            require     => [ Exec["ros-aptupdate"], Package["python-rosdep"], File["/opt/ros/${distribution}"], ],
        } ->
        file { "/etc/profile.d/30-ros-env.sh":
            ensure      => present,
            mode        => "644",
            owner       => "root",
            group       => "root",
            content     => "source /opt/ros/${distribution}/setup.bash",
        } ->
        # Initialize rosdep
        exec { "rosdep-init":
            command         => "/usr/bin/rosdep init",
            creates         => "/etc/ros/rosdep/sources.list.d/20-default.list",
        } ->
        exec { "rosdep-update":
            user            => "mav",
            command         => "/usr/bin/rosdep update",
            creates         => "/srv/maverick/.ros/rosdep/sources.cache",
        }
        
    # Build from source
    } elsif $_installtype == "source" {

        if ! ("install_flag_ros" in $installflags) {
            file { "${builddir}":
                ensure      => directory,
                owner       => "mav",
                group       => "mav",
                mode        => "755",
            }
            
            $buildparallel = ceiling((1 + $::processorcount) / 2) # Restrict build parallelization to roughly processors/2 if raspberry
            ensure_packages(["build-essential"])
            # Install ros install packages
            package {["python-rosinstall", "python-rosinstall-generator"]:
                require         => Exec["ros-repo"]
            } ->
            # Initialize rosdep
            exec { "rosdep-init":
                command         => "/usr/bin/rosdep init",
                creates         => "/etc/ros/rosdep/sources.list.d/20-default.list",
                require         => [ Package["python-rosdep"], Package["python-wstool"], Package["python-rosinstall"], Package["python-rosinstall-generator"] ]
            } ->
            exec { "rosdep-update":
                user            => "mav",
                command         => "/usr/bin/rosdep update",
                creates         => "/srv/maverick/.ros/rosdep/sources.cache",
                require         => Package["python-rosdep"]
            } ->
            exec { "catkin_rosinstall":
                environment     => ["CMAKE_PREFIX_PATH=/srv/maverick/software/opencv"],
                command         => "/usr/bin/rosinstall_generator ${buildtype} --rosdistro ${distribution} --deps --wet-only --tar > ${distribution}-${buildtype}-wet.rosinstall && /usr/bin/wstool init -j${buildparallel} src ${distribution}-${buildtype}-wet.rosinstall",
                cwd             => "${builddir}",
                user            => "mav",
                creates         => "${builddir}/src/.rosinstall"
            } ->
            exec { "catkin_make":
                environment     => ["CMAKE_PREFIX_PATH=/srv/maverick/software/opencv"],
                command         => "/usr/bin/rosdep install --from-paths src --ignore-src --rosdistro ${distribution} -y && ${builddir}/src/catkin/bin/catkin_make_isolated --install --install-space ${installdir}/${distribution} -DCMAKE_BUILD_TYPE=Release -j${buildparallel}",
                cwd             => "${builddir}",
                user            => "mav",
                creates         => "${installdir}/${distribution}/lib/rosbag/topic_renamer.py",
                timeout         => 0,
                require         => File["${installdir}/${distribution}"]
            } ->
            file { "/srv/maverick/var/build/.install_flag_ros":
                ensure      => present,
            }
            
            if $module_opencv == true {
                # Add opencv to the existing workspace through vision_opencv package
                ensure_packages(["libpoco-dev", "libyaml-cpp-dev"])
                exec { "ws_add_opencv":
                    command         => "/usr/bin/rosinstall_generator vision_opencv --rosdistro ${distribution} --deps --wet-only --tar >${distribution}-vision_opencv-wet.rosinstall && /usr/bin/wstool merge -t src ${distribution}-vision_opencv-wet.rosinstall && /usr/bin/wstool update -t src",
                    cwd             => "${builddir}",
                    user            => "mav",
                    creates         => "${builddir}/src/vision_opencv",
                    timeout         => 0,
                    environment => ["PKG_CONFIG_PATH=/srv/maverick/software/gstreamer/lib/pkgconfig:/srv/maverick/software/opencv/lib/pkgconfig"],
                    require         => [ Package["libpoco-dev"], Exec["catkin_make"] ]
                } ->
                exec { "catkin_make_vision_opencv":
                    command         => "${builddir}/src/catkin/bin/catkin_make_isolated --install --install-space ${installdir}/${distribution} -DCMAKE_BUILD_TYPE=Release -j${buildparallel}",
                    cwd             => "${builddir}",
                    user            => "mav",
                    creates         => "${installdir}/${distribution}/lib/libopencv_optflow3.so",
                    environment => ["PKG_CONFIG_PATH=/srv/maverick/software/gstreamer/lib/pkgconfig:/srv/maverick/software/opencv/lib/pkgconfig"],
                    timeout         => 0,
                    require         => File["${installdir}/${distribution}"]
                }
            }
            
            if $module_mavros == true {
                # Add mavros to the existing workspace, this also installs mavlink package as dependency
                exec { "ws_add_mavros":
                    command         => "/usr/bin/rosinstall_generator mavros --rosdistro ${distribution} --deps --wet-only --tar >${distribution}-mavros-wet.rosinstall && /usr/bin/rosinstall_generator visualization_msgs --rosdistro ${distribution} --deps --wet-only --tar >>${distribution}-mavros-wet.rosinstall && /usr/bin/rosinstall_generator urdf --rosdistro ${distribution} --deps --wet-only --tar >>${distribution}-mavros-wet.rosinstall && /usr/bin/wstool merge -t src ${distribution}-mavros-wet.rosinstall && /usr/bin/wstool update -t src",
                    cwd             => "${builddir}",
                    user            => "mav",
                    creates         => "${builddir}/src/mavros",
                    environment => ["PKG_CONFIG_PATH=/srv/maverick/software/gstreamer/lib/pkgconfig:/srv/maverick/software/opencv/lib/pkgconfig"],
                    timeout         => 0,
                    require         => Exec["catkin_make"]
                } ->
                exec { "catkin_make_mavros":
                    # Note must only use -j1 otherwise we get compiler errors
                    command         => "/usr/bin/rosdep install --from-paths src --ignore-src --rosdistro ${distribution} -y && ${builddir}/src/catkin/bin/catkin_make_isolated --install --install-space ${installdir}/${distribution} -DCMAKE_BUILD_TYPE=Release -j1",
                    cwd             => "${builddir}",
                    user            => "mav",
                    creates         => "${installdir}/${distribution}/lib/libmavros.so",
                    environment => ["PKG_CONFIG_PATH=/srv/maverick/software/gstreamer/lib/pkgconfig:/srv/maverick/software/opencv/lib/pkgconfig"],
                    timeout         => 0,
                    require         => File["${installdir}/${distribution}"]
                }
            }
        } else {
            # If we don't build ros, we still need something for other manifest dependencies
            file { "/srv/maverick/var/build/.install_flag_ros":
                ensure      => present,
            }
        }

        file { "/etc/profile.d/30-ros-env.sh":
            ensure      => present,
            mode        => "644",
            owner       => "root",
            group       => "root",
            content     => "source /opt/ros/${distribution}/setup.bash",
            require         => File["/srv/maverick/var/build/.install_flag_ros"],
        }

    }
    
    # Install rosmaster systemd manifest.  Note it's not activated here, other modules will call the rosmaster define
    file { "/etc/systemd/system/maverick-rosmaster@.service":
        ensure          => present,
        source          => "puppet:///modules/maverick_ros/maverick-rosmaster@.service",
        owner           => "root",
        group           => "root",
        mode            => "644",
        notify          => Exec["maverick-systemctl-daemon-reload"],
    }
    # Create directory for ros config
    file { "/srv/maverick/data/config/ros":
        ensure          => directory,
        owner           => "mav",
        group           => "mav",
        mode            => "755",
    }
    # Create a symlink to rosmaster launch script
    file { "/srv/maverick/software/maverick/bin/rosmaster.sh":
        ensure      => link,
        target      => "/srv/maverick/software/maverick/manifests/maverick-modules/maverick_ros/files/rosmaster.sh",
    }
    
    # Create log directories
    file { ["/srv/maverick/var/log/ros", "/srv/maverick/var/log/ros/fc", "/srv/maverick/var/log/ros/sitl"]:
        ensure      => directory,
        mode        => "755",
        owner       => "mav",
        group       => "mav",
    }

    # Install mavros systemd manifest.  Like rosmaster, it's not activated here but used by mavros define
    file { "/etc/systemd/system/maverick-mavros@.service":
        source      => "puppet:///modules/maverick_ros/maverick-mavros@.service",
        owner       => "root",
        group       => "root",
        mode        => "644",
        notify      => Exec["maverick-systemctl-daemon-reload"],
    }
    # Create a symlink to mavros launch script
    file { "/srv/maverick/software/maverick/bin/mavros.sh":
        ensure      => link,
        target      => "/srv/maverick/software/maverick/manifests/maverick-modules/maverick_ros/files/mavros.sh",
    }

}