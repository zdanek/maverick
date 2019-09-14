class maverick_ros::ros2 (
    $installtype = "auto",
    $distribution = "auto",
    $builddir = "/srv/maverick/var/build/ros2",
    $installdir = "/srv/maverick/software/ros2",
    $metapackage = "desktop", # desktop or ros-base
) {

    # If installtype is set then use it and skip autodetection
    if $installtype == "native" {
        if $ros2_installed == "no" { 
            notice("ROS2: Native installation requested")
        }
        $_installtype = "native"
    } elsif $installtype == "source" {
        if $ros2_installed == "no" {
            notice("ROS2: Source installation requested")
        }
        $_installtype = "source"
    # First try and determine build type based on OS and architecture
    } elsif $installtype == "auto" {
        case $::operatingsystem {
            "Ubuntu": {
                case $::lsbdistcodename {
                    "xenial": {
                        $autodist = "crystal"
                        case $architecture {
                            "amd64", "arm64", "aarch64": { $_installtype = "native" }
                            default: { $_installtype = "source" }
                        }
                    }
                    "bionic": {
                        $autodist = "dashing"
                        case $architecture {
                            "amd64", "arm64", "aarch64": { $_installtype = "native" }
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
                        $autodist = "crystal"
                        $_installtype = "source"
                    }
                    "9": { # stretch
                        $autodist = "dashing"
                        $_installtype = "source"
                    }
                    default: {
                        $autodist = undef
                        $_installtype = undef
                    }
                }
            }
        }
        if $_installtype == "native" and $ros2_installed == "no" {
            notice("ROS2: supported platform detected for ${autodist} distribution, using native packages")
        } elsif $_installtype == "source" and $ros2_installed == "no" {
            notice("ROS2: unsupported platform for ${autodist} distribution, installing from source")
        }
    } else {
        $_installtype = false
    }
    
    if $distribution == "auto" and $installtype == "auto" {
        $_distribution = $autodist
    } else {
        $_distribution = $distribution
    }

    if $_installtype and $_distribution {
        # Install dependencies
        
        # Work out distro name
        if $::lsbdistid == "ubilinux" and $::lsbmajdistrelease == "4" {
            $_distro = "stretch"
        } else {
            $_distro = $::lsbdistcodename
        }
        
        # Create symlink to usual vendor install directory
        if ! defined(File["/opt/ros"]) {
            file { ["/opt/ros"]:
                ensure      => directory,
                mode        => "755",
                owner       => "root",
                group       => "root",
            }
        }
        file { ["${installdir}", "${installdir}/${_distribution}"]:
            ensure      => directory,
            owner       => "mav",
            group       => "mav",
            mode        => "755",
            require     => File["/opt/ros"],
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
        }

        # Install python module that provides autocomplete        
        install_python_module { "ros2-argcomplete":
            pkgname     => "argcomplete",
            ensure      => present,
        }

        # Install ROS bootstrap from ros.org packages
        apt::key { 'ros2-repo-key':
            id      => 'C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654',
            server  => 'keyserver.ubuntu.com',
            require     => Package["dirmngr"],
            notify      => Exec["ros_apt_update"],            
        } ->
        exec { "ros2-repo":
            command     => "/bin/echo \"deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu ${_distro} main\" > /etc/apt/sources.list.d/ros2-latest.list",
            unless      => "/bin/grep '${_distro}' /etc/apt/sources.list.d/ros2-latest.list",
            notify      => Exec["ros2_apt_update"],
        } ->
        exec { "ros2_apt_update":
            command     => "/usr/bin/apt update",
            refreshonly => true,
        }
    }

    # Install from ros repos
    if $_installtype == "native" {
        package { "ros-${_distribution}-${metapackage}":
            ensure      => present,
            require     => Exec["ros2_apt_update"],
        } ->
        package { "ros-${_distribution}-ros1-bridge":
            ensure      => present,
        } ->
        package { ["ros-${_distribution}-cv-bridge", "ros-${_distribution}-vision-opencv", "ros-${_distribution}-image-geometry"]:
            ensure      => present,
        }
    } elsif $_installtype == "source" {
        # Force osdistro for raspberry
        if $raspberry_present == "yes" {
            $_osdistro = "--os=debian:${::lsbdistcodename}"
        } elsif $::operatingsystem == "Ubuntu" {
            $_osdistro = "--os=ubuntu:${::lsbdistcodename}"
        } elsif $::operatingsystem == "Debian" {
            $_osdistro = "--os=debian:${::lsbdistcodename}"
        } else {
            $_osdistro = ""
        }

        if ! ("install_flag_ros2" in $installflags) {
            file { ["${builddir}", "${builddir}/src", "/etc/ros", "/etc/ros/rosdep"]:
                ensure      => directory,
                owner       => "mav",
                group       => "mav",
                mode        => "755",
            }
            
            $buildparallel = ceiling((1 + $::processorcount) / 2) # Restrict build parallelization to roughly processors/2
            install_python_module { ["colcon-common-extensions", "rosdep", "vcstool"]:
                ensure  => present,
            } ->
            package { ["libasio-dev", "libtinyxml2-dev"]:
                ensure  => present,
            } ->
            exec { "ros2-src-repo":
                cwd     => "${builddir}",
                command => "/usr/bin/wget https://raw.githubusercontent.com/ros2/ros2/dashing/ros2.repos",
                creates => "${builddir}/ros2.repos",
                user    => "mav",
            } ->
            exec { "ros2-vcs-import":
                cwd     => "${builddir}",
                command => "/srv/maverick/software/python/bin/vcs import src <ros2.repos",
                creates => "${builddir}/src/ros2",
                user    => "mav",
            } ->
            exec { "ros2-rosdep-init":
                cwd     => "${builddir}",
                command => "/srv/maverick/software/python/bin/rosdep init",
                creates => "/etc/ros/rosdep/sources.list.d/20-default.list",
                user    => "mav",
            } ->
            exec { "ros2-rosdep-update":
                cwd     => "${builddir}",
                command => "/srv/maverick/software/python/bin/rosdep update",
                user    => "mav",
                creates => "/srv/maverick/.ros/rosdep/sources.cache",
            } ->
            exec { "ros2-rosdep-install":
                cwd     => "${builddir}",
                command => "/srv/maverick/software/python/bin/rosdep install --from-paths src --ignore-src --rosdistro ${_distribution} ${_osdistro} -y --skip-keys 'console_bridge fastcdr fastrtps libopensplice67 libopensplice69 rti-connext-dds-5.3.1 urdfdom_headers'",
                user    => "mav",
            } ->
            exec { "ros2-colcon build":
                cwd     => "${builddir}",
                environment => ["PYTHON_EXECUTABLE=/srv/maverick/software/python/bin/python3"],
                command => "/srv/maverick/software/python/bin/colcon build --cmake-args -DBUILD_TESTING=0 -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/srv/maverick/software/python/bin/python3 --catkin-skip-building-tests --symlink-install --packages-skip ros1_bridge >/srv/maverick/var/log/build/ros2.colcon.build 2>&1",
                #command => "/srv/maverick/software/python/bin/colcon build --cmake-args -DBUILD_TESTING=0 -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/srv/maverick/software/python/bin/python3 --catkin-skip-building-tests --symlink-install --packages-skip python_qt_binding qt_gui qt_gui_py_common qt_gui_cpp qt_dotgraph qt_gui_app ros1_bridge >/srv/maverick/var/log/build/ros2.colcon.build 2>&1",
                #creates => "/etc/ros/rosdep/sources.list.d/20-default.list",
                user    => "mav",
                timeout => 0,
            }
        }
    }

}
