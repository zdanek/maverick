class maverick_ros::ros1 (
    $installtype = "auto",
    $distribution = "auto",
    $buildtype = "ros_comm", # ROS core variant, ros_comm is base without GUI, can also be desktop, desktop_full.  mobile and perception variants useful for drones.  desktop_full includes everything.
    $builddir = "/srv/maverick/var/build/ros_catkin_ws",
    $installdir = "/srv/maverick/software/ros",
    $module_mavros = true,
    $module_realsense = true,
    $module_opencv = true,
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
    } elsif $installtype == "auto" {
        case $::operatingsystem {
            "Ubuntu": {
                case $::lsbdistcodename {
                    "wily": {
                        $autodist = "kinetic"
                        case $architecture {
                            "amd64", "i386": { $_installtype = "native" }
                            default: { $_installtype = "source" }
                        }
                    }
                    "xenial": {
                        $autodist = "kinetic"
                        case $architecture {
                            "amd64", "i386", "armhf", "arm64", "aarch64": { $_installtype = "native" }
                            default: { $_installtype = "source" }
                        }
                    }
                    "yakkety": {
                        $autodist = "lunar"
                        case $architecture {
                            "amd64": { $_installtype = "native" }
                            default: { $_installtype = "source" }
                        }
                    }
                    "zesty": {
                        $autodist = "lunar"
                        case $architecture {
                            "amd64": { $_installtype = "native" }
                            default: { $_installtype = "source" }
                        }
                    }
                    "bionic": {
                        $autodist = "melodic"
                        case $architecture {
                            "amd64", "armhf", "arm64", "aarch64": { $_installtype = "native" }
                            default: { $_installtype = "source" }
                        }
                    }
                    default: {
                        $autodist = undef
                        $_installtype = undef
                    }
                }
            }
            "Debian": {
                case $::operatingsystemmajrelease {
                    # For Debian OS use version number instead of codename, for derivatives like rasbian and ubilinux
                    "8": { # jessie
                        $autodist = "kinetic"
                        case $::architecture {
                            "amd64", "arm64": { $_installtype = "native" }
                            default: { $_installtype = "source" }
                        }
                    }
                    "9": { # stretch
                        case $::raspberry_present {
                            "yes": { $autodist = "melodic" }
                            default: { $autodist = "lunar" }
                        }
                        case $::architecture {
                            "amd64", "arm64": { $_installtype = "native" }
                            default: { $_installtype = "source" }
                        }
                    }
                    "10": { # buster
                        case $::raspberry_present {
                            "yes": { $autodist = "melodic" }
                            default: { $autodist = "melodic" }
                        }
                        case $::architecture {
                            "amd64", "arm64": { $_installtype = "source" }
                            default: { $_installtype = "source" }
                        }
                    }
                    default: {
                        $autodist = undef
                        $_installtype = undef
                    }
                }
            }
        }
        if $_installtype == "native" and $ros_installed == "no" {
            notice("ROS: supported platform detected for ${autodist} distribution, using native packages")
        } elsif $_installtype == "source" and $ros_installed == "no" {
            notice("ROS: unsupported platform for ${autodist} distribution, installing from source")
        }
    } else {
        $_installtype = false
    }
    
    if $distribution == "auto" and $installtype == "auto" {
        $_distribution = $autodist
    } else {
        $_distribution = $distribution
    }
    
    if $installtype and $_distribution {
        # Install dependencies
        ensure_packages(["dirmngr"])
        ensure_packages(["libpoco-dev", "libyaml-cpp-dev"])
        # Work out distro name
        if $::lsbdistid == "ubilinux" and $::lsbmajdistrelease == "4" {
            $_distro = "stretch"
        } else {
            $_distro = $::lsbdistcodename
        }
        # Create symlink to usual vendor install directory
        file { ["/opt", "/opt/ros"]:
            ensure      => directory,
            mode        => "755",
            owner       => "root",
            group       => "root",
        } ->
        file { ["${installdir}", "${installdir}/${_distribution}"]:
            ensure      => directory,
            owner       => "mav",
            group       => "mav",
            mode        => "755",
        } ->
        file { "/opt/ros/${_distribution}":
            ensure      => link,
            target      => "${installdir}/${_distribution}",
            force       => true,
        } ->
        file { "${installdir}/current":
            ensure      => link,
            target      => "${installdir}/${_distribution}",
            force       => true,
        } ->
        file { "/opt/ros/current":
            ensure      => link,
            target      => "${installdir}/${_distribution}",
            force       => true,
        } ->
        # Remove compromised ROS repo key
        apt::key { 'ros-repo-badkey':
            id      => '421C365BD9FF1F717815A3895523BAEEB01FA116',
            ensure  => absent,
            require     => Package["dirmngr"],
        } ->
        # Install ROS bootstrap from ros.org packages
        exec { "ros-repo":
            command     => "/bin/echo \"deb http://packages.ros.org/ros/ubuntu ${_distro} main\" > /etc/apt/sources.list.d/ros-latest.list",
            unless      => "/bin/grep '${_distro}' /etc/apt/sources.list.d/ros-latest.list",
            notify      => Exec["ros_apt_update"],
            require     => Apt_key['ros-repo-key'],
        } ->
        exec { "ros_apt_update":
            command     => "/usr/bin/apt update",
            refreshonly => true,
        } ->
        package { ["python-rosdep", "python-catkin-tools"]:
            ensure      => installed,
            require     => Exec["apt_update"],
        }
        ensure_packages(["python-wstool", "python-wstools"])
    }

    # Install from ros repos
    if $_installtype == "native" {
        package { ["ros-${_distribution}-perception", "ros-${_distribution}-viz", "ros-${_distribution}-mavros", "ros-${_distribution}-mavros-extras", "ros-${_distribution}-mavros-msgs", "ros-${_distribution}-test-mavros"]:
            ensure      => installed,
            require     => [ Exec["ros_apt_update"], Package["python-rosdep"], File["/opt/ros/${_distribution}"], ],
        } ->
        # Download latest geographiclib install script
        exec { "download_geoinstall":
            command         => "/usr/bin/wget -O /srv/maverick/software/ros/${_distribution}/lib/mavros/install_geographiclib_datasets.master.sh https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh",
            creates         => "/srv/maverick/software/ros/${_distribution}/lib/mavros/install_geographiclib_datasets.master.sh",
        } ->
        # Install mavros geographiclib dependencies
        exec { "mavros_geoinstall":
            command         => "/bin/bash /srv/maverick/software/ros/${_distribution}/lib/mavros/install_geographiclib_datasets.master.sh >/srv/maverick/var/log/build/ros.mavros_geoinstall.out 2>&1",
            creates         => "/usr/share/GeographicLib/geoids/egm96-5.pgm",
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
        } ->
        file { "/etc/profile.d/30-ros-env.sh":
            ensure      => present,
            mode        => "644",
            owner       => "root",
            group       => "root",
            content     => "source /opt/ros/${_distribution}/setup.bash",
        } ->
        # Install python3 packages
        package { ["python3-rospkg-modules", "python3-catkin-pkg-modules"]:
            ensure      => present,
            require     => Exec["ros_apt_update"],
        } ->
        exec { "ros-install-marker":
            command     => "/bin/false",
            refreshonly => true,
        }

    # Build from source
    } elsif $_installtype == "source" {

        # Force osdistro for raspberry
        if $raspberry_present == "yes" {
            $_osdistro = "--os=debian:${::lsbdistcodename}"
        } else {
            $_osdistro = ""
        }

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
                require         => [ Package["python-rosdep"], Package["python-wstool"], Package["python-rosinstall"], Package["python-rosinstall-generator"] ],
            } ->
            exec { "rosdep-update":
                user            => "mav",
                command         => "/usr/bin/rosdep update",
                creates         => "/srv/maverick/.ros/rosdep/sources.cache",
                require         => Package["python-rosdep"]
            } ->
            exec { "catkin_rosinstall":
                environment     => ["CMAKE_PREFIX_PATH=/srv/maverick/software/opencv"],
                command         => "/usr/bin/rosinstall_generator ${buildtype} --rosdistro ${_distribution} --deps --tar > ${_distribution}-${buildtype}.rosinstall && /usr/bin/wstool init -j${buildparallel} src ${_distribution}-${buildtype}.rosinstall >/srv/maverick/var/log/build/ros.rosinstall.out 2>&1",
                cwd             => "${builddir}",
                user            => "mav",
                creates         => "${builddir}/src/.rosinstall"
            } ->
            exec { "catkin_make":
                environment     => ["CMAKE_PREFIX_PATH=/srv/maverick/software/opencv"],
                command         => "/usr/bin/rosdep install --from-paths src --ignore-src --rosdistro ${_distribution} -y ${_osdistro} >/srv/maverick/var/log/build/ros.rosdep.out 2>&1 && ${builddir}/src/catkin/bin/catkin_make_isolated --install --install-space ${installdir}/${_distribution} -DCMAKE_BUILD_TYPE=Release -j${buildparallel} >/srv/maverick/var/log/build/ros.catkin_make.out 2>&1",
                cwd             => "${builddir}",
                user            => "mav",
                creates         => "${installdir}/${_distribution}/lib/rosbag/topic_renamer.py",
                timeout         => 0,
                require         => File["${installdir}/${_distribution}"],
            } ->
            file { "/etc/profile.d/30-ros-env.sh":
                ensure      => present,
                mode        => "644",
                owner       => "root",
                group       => "root",
                content     => "source /opt/ros/${_distribution}/setup.bash",
            } ->
            file { "/srv/maverick/var/build/.install_flag_ros":
                ensure      => present,
            }
        }
        exec { "ros-install-marker":
            command     => "/bin/false",
            refreshonly => true,
        }
        
        if $module_mavros == true {
            if ! ("install_flag_ros_mavros" in $installflags) {
                file { ["/srv/maverick/var/build/catkin_ws_mavros", "/srv/maverick/var/build/catkin_ws_mavros/src"]:
                    ensure  => directory,
                    owner   => "mav",
                } ->
                exec { "ros-mavros-catkin-init":
                    user        => "mav",
                    cwd         => "/srv/maverick/var/build/catkin_ws_mavros",
                    command     => "/usr/bin/catkin config --init --extend /opt/ros/current --install -i /opt/ros/current",
                    environment => ["PYTHONPATH=/opt/ros/current/lib/python2.7/dist-packages", "CMAKE_PREFIX_PATH=/opt/ros/melodic:/srv/maverick/software/realsense-sdk2"],
                    creates     => "/srv/maverick/var/build/catkin_ws_mavros/.catkin_tools/profiles/default/config.yaml",
                } ->
                exec { "ros-mavros-wstool-init":
                    user        => "mav",
                    cwd         => "/srv/maverick/var/build/catkin_ws_mavros",
                    command     => "/usr/bin/wstool init src",
                    creates     => "/srv/maverick/var/build/catkin_ws_mavros/src/.rosinstall",
                } ->
                exec { "ros-mavros-rosinstall_mavlink":
                    user        => "mav",
                    cwd         => "/srv/maverick/var/build/catkin_ws_mavros",
                    command     => "/usr/bin/rosinstall_generator --rosdistro ${_distribution} mavlink >/srv/maverick/var/build/catkin_ws_mavros/mavros.rosinstall",
                    creates     => "/srv/maverick/var/build/catkin_ws_mavros/mavros.rosinstall",
                } ->
                exec { "ros-mavros-rosinstall_mavros":
                    user        => "mav",
                    cwd         => "/srv/maverick/var/build/catkin_ws_mavros",
                    command     => "/usr/bin/rosinstall_generator --rosdistro ${_distribution} --upstream-development mavros mavros_extras mavros_msgs test_mavros sensor_msgs control_toolbox realtime_tools python_orocos_kdl urdf >>/srv/maverick/var/build/catkin_ws_mavros/mavros.rosinstall",
                    unless      => "/bin/grep mavros mavros.rosinstall",
                } ->
                exec { "ros-mavros-wstool-merge":
                    user        => "mav",
                    cwd         => "/srv/maverick/var/build/catkin_ws_mavros",
                    command     => "/usr/bin/wstool merge -t src mavros.rosinstall",
                    creates     => "/srv/maverick/var/build/catkin_ws_mavros/src/.rosinstall.bak",
                } ->
                exec { "ros-mavros-wstool-update":
                    user        => "mav",
                    cwd         => "/srv/maverick/var/build/catkin_ws_mavros",
                    command     => "/usr/bin/wstool update -t src -j4",
                    creates     => "/srv/maverick/var/build/catkin_ws_mavros/src/mavros/mavros",
                } ->
                # Install mavros geographiclib dependencies
                exec { "mavros_geoinstall":
                    command         => "/bin/bash /srv/maverick/software/ros/${_distribution}/lib/mavros/install_geographiclib_datasets.sh >/srv/maverick/var/log/build/ros.mavros_geoinstall.out 2>&1",
                    creates         => "/usr/share/GeographicLib/geoids/egm96-5.pgm",
                } ->
                package { "python-sip-dev": 
                    ensure      => present,
                } ->
                exec { "ros-mavros-catkin-build":
                    user        => "mav",
                    cwd         => "/srv/maverick/var/build/catkin_ws_mavros",
                    command     => "/usr/bin/catkin build -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release >/srv/maverick/var/log/build/mavros-catkin-build.out 2>&1",
                    creates     => "/srv/maverick/var/build/catkin_ws_mavros/src/mavros/mavros >/var/",
                    timeout     => 0,
                    #before      => File["/opt/ros/${_distribution}/share/mavros/launch/apm_config.yaml"],
                } ->
                file { "/srv/maverick/var/build/.install_flag_ros_mavros":
                    ensure      => present,
                }
            }
        }

        if $module_opencv == true {
            if ! ("install_flag_ros_opencv" in $installflags) {
                file { ["/srv/maverick/var/build/catkin_ws_opencv", "/srv/maverick/var/build/catkin_ws_opencv/src"]:
                    ensure  => directory,
                    owner   => "mav",
                } ->
                exec { "ros-opencv-catkin-init":
                    user        => "mav",
                    cwd         => "/srv/maverick/var/build/catkin_ws_opencv",
                    command     => "/usr/bin/catkin config --init --extend /opt/ros/current --install -i /opt/ros/current",
                    environment => ["PYTHONPATH=/opt/ros/current/lib/python2.7/dist-packages", "CMAKE_PREFIX_PATH=/opt/ros/melodic:/srv/maverick/software/opencv"],
                    creates     => "/srv/maverick/var/build/catkin_ws_opencv/.catkin_tools/profiles/default/config.yaml",
                } ->
                exec { "ros-opencv-wstool-init":
                    user        => "mav",
                    cwd         => "/srv/maverick/var/build/catkin_ws_opencv",
                    command     => "/usr/bin/wstool init src",
                    creates     => "/srv/maverick/var/build/catkin_ws_opencv/src/.rosinstall",
                } ->
                exec { "ros-opencv-rosinstall_opencv":
                    user        => "mav",
                    cwd         => "/srv/maverick/var/build/catkin_ws_opencv",
                    command     => "/usr/bin/rosinstall_generator --rosdistro ${_distribution} vision_opencv cv_bridge image_geometry >/srv/maverick/var/build/catkin_ws_opencv/opencv.rosinstall",
                    creates     => "/srv/maverick/var/build/catkin_ws_opencv/opencv.rosinstall",
                } ->
                exec { "ros-opencv-wstool-merge":
                    user        => "mav",
                    cwd         => "/srv/maverick/var/build/catkin_ws_opencv",
                    command     => "/usr/bin/wstool merge -t src opencv.rosinstall",
                    creates     => "/srv/maverick/var/build/catkin_ws_opencv/src/.rosinstall.bak",
                } ->
                exec { "ros-opencv-wstool-update":
                    user        => "mav",
                    cwd         => "/srv/maverick/var/build/catkin_ws_opencv",
                    command     => "/usr/bin/wstool update -t src -j4",
                    creates     => "/srv/maverick/var/build/catkin_ws_opencv/src/cv_bridge",
                } ->
                exec { "ros-opencv-catkin-build":
                    user        => "mav",
                    cwd         => "/srv/maverick/var/build/catkin_ws_opencv",
                    command     => "/usr/bin/catkin build -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release >/srv/maverick/var/log/build/opencv-catkin-build.out 2>&1",
                    creates     => "/srv/maverick/var/build/catkin_ws_opencv/src/cv_bridge >/var/",
                    timeout     => 0,
                } ->
                file { "/srv/maverick/var/build/.install_flag_ros_opencv":
                    ensure      => present,
                }
            }
        }

    }
  
    if $module_realsense == true {
        if ! ("install_flag_ros_realsense" in $installflags) {
            if "install_flag_ros" in $installflags {
                $_realsense_deps = undef
            } else {
                $_realsense_deps = Exec["catkin_make"]
            }
            file { ["/srv/maverick/var/build/catkin_ws_realsense", "/srv/maverick/var/build/catkin_ws_realsense/src"]:
                ensure  => directory,
                owner   => "mav",
            } ->
            package { "ros-${_distribution}-ddynamic-reconfigure":
               ensure   => installed,
               require  => Exec["ros_apt_update"],
            } ->
            package { ["ros-${_distribution}-nodelet", "ros-${_distribution}-nodelet-core", "ros-${_distribution}-nodelet-topic-tools"]:
                ensure  => installed,
            } ->
            oncevcsrepo { "git-ros-realsense":
                gitsource   => "https://github.com/IntelRealSense/realsense-ros.git",
                dest        => "/srv/maverick/var/build/catkin_ws_realsense/src/realsense-ros",
                revision    => "2.2.8",
                depth       => undef,
            } ->
            exec { "ros-realsense-init-workspace":
                user        => "mav",
                cwd         => "/srv/maverick/var/build/catkin_ws_realsense/src",
                command     => "/opt/ros/current/bin/catkin_init_workspace",
                environment => ["PYTHONPATH=/opt/ros/current/lib/python2.7/dist-packages"],
                creates     => "/srv/maverick/var/build/catkin_ws_realsense/src/CMakeLists.txt",
            } ->
            exec { "ros-realsense-catkin-make":
                user        => "mav",
                cwd         => "/srv/maverick/var/build/catkin_ws_realsense",
                command     => "/opt/ros/current/bin/catkin_make clean; /opt/ros/current/bin/catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${installdir}/${_distribution} >/srv/maverick/var/log/build/ros.realsense.catkin_make.out 2>&1",
                timeout     => 0,
                environment => ["PYTHONPATH=/opt/ros/current/lib/python2.7/dist-packages", "CMAKE_PREFIX_PATH=/opt/ros/melodic:/srv/maverick/software/realsense-sdk2"],
                creates     => "/srv/maverick/var/build/catkin_ws_realsense/build/realsense-ros/realsense2_camera/catkin_generated/installspace/realsense2_camera.pc",
                require     => Exec["ros-install-marker"],
            } ->
            exec { "ros-realsense-catkin-install":
                cwd         => "/srv/maverick/var/build/catkin_ws_realsense",
                command     => "/opt/ros/current/bin/catkin_make install -DCMAKE_INSTALL_PREFIX=${installdir}/${_distribution} >/srv/maverick/var/log/build/ros.realsense.catkin_install.out 2>&1",
                timeout     => 0,
                environment => ["PYTHONPATH=/opt/ros/current/lib/python2.7/dist-packages", "CMAKE_PREFIX_PATH=/opt/ros/melodic:/srv/maverick/software/realsense-sdk2"],
                creates     => "/srv/maverick/software/ros/current/lib/librealsense2_camera.so",
            }
            file { "/srv/maverick/var/build/.install_flag_ros_realsense":
                ensure      => present,
            }
        }
    }

    if $installtype and $_distribution {
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
        file { "/srv/maverick/config/ros":
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
        file { ["/srv/maverick/var/log/ros", "/srv/maverick/var/log/ros/fc"]:
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
        # Install a fixed apm_config.yaml
        # https://github.com/mavlink/mavros/issues/1210
        #file { "/opt/ros/${_distribution}/share/mavros/launch/apm_config.yaml":
        #    content     => template("maverick_ros/apm_config.yaml.erb")
        #}
    }

    # Install python deps
    install_python_module { "pip-defusedxml":
        pkgname     => "defusedxml",
        ensure      => present,
    }
    install_python_module { "pip-rospkg":
        pkgname     => "rospkg",
        ensure      => present,
    }
}
