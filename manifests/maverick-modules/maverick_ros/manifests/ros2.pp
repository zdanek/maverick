# @summary
#   Maverick_ros::Ros2 class
#   This class installs/manages ROS2 (ros.org).
#
# @example Declaring the class
#   This class is included from maverick_ros class and should not be included from elsewhere
#
# @param installtype
#   If 'auto' then the type of install needed will be detected based on the operating system version and architecture.  This can be overidden with 'native' or 'source'.
# @param distribution
#   Specify the ROS distribution to use - eg. kinetic, melodic
# @param builddir
#   Set the build location.
# @param installdir
#   Set the install location, should always be /srv/maverick/sofware/ros2 (which is symlinked to /opt/ros2) in Maverick
# @param metapackage
#   The metapackage to use for binary install.
# @param ros1_bridge
#   If true, build and install the ROS1 bridge.
#
class maverick_ros::ros2 (
    Enum['native', 'source', 'auto'] $installtype = "auto",
    String $distribution = "auto",
    String $builddir = "/srv/maverick/var/build/ros2",
    String $installdir = "/srv/maverick/software/ros2",
    String $metapackage = "ros-base", # desktop or ros-base
    Boolean $ros1_bridge = true,
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
                        $autodist = "eloquent"
                        case $architecture {
                            "amd64", "arm64", "aarch64", "armhf": { $_installtype = "native" }
                            default: { $_installtype = "source" }
                        }
                    }
                    "focal": {
                        $autodist = "foxy"
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
            "Debian", "Raspbian": {
                case $::operatingsystemmajrelease {
                    # For Debian OS use version number instead of codename, for derivatives like rasbian and ubilinux
                    "8": { # jessie
                        $autodist = "eloquent"
                        $_installtype = "source"
                    }
                    "9": { # stretch
                        $autodist = "eloquent"
                        $_installtype = "source"
                    }
                    "10": { # buster
                        $autodist = "foxy"
                        $_installtype = "source"
                    }
                    "11": { # bullseye
                        if $ros1_bridge == true {
                            $_ros1_bridge = false
                            warning("ROS2: ROS1 bridge not supported on ${::operatingsystem} ${::operatingsystemmajrelease}")
                        }
                        $autodist = "iron"
                        $_installtype = "source"
                    }
                    "12": { # bookworm
                        if $ros1_bridge == true {
                            $_ros1_bridge = false
                            warning("ROS2: ROS1 bridge not supported on ${::operatingsystem} ${::operatingsystemmajrelease}")
                        }
                        $autodist = "humble"
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
        $_installtype = undef
    }

    if !$_installtype {
        fail("ROS2: Cannot decide which ROS2 install, on ${::operatingsystem}, ${::operatingsystemmajrelease}, ${::architecture}, Install type: ${installtype}, Distribution: ${distribution}, Build type: ${buildtype}")
    }

    if $_ros1_bridge == undef {
        $_ros1_bridge = $ros1_bridge
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
        if ! defined(File["${installdir}"]) {
            file { ["${installdir}"]:
                ensure      => directory,
                owner       => "mav",
                group       => "mav",
                mode        => "755",
                require     => File["/opt/ros"],
            }
        }
        file { "${installdir}/${_distribution}":
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
        }
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
        exec { "ros2-repo":
            command     => "/bin/echo \"deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu ${_distro} main\" > /etc/apt/sources.list.d/ros2-latest.list",
            unless      => "/bin/grep '${_distro}' /etc/apt/sources.list.d/ros2-latest.list",
            notify      => Exec["ros2_apt_update"],
            require     => Apt_key['ros-repo-key'],
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
            require     => Exec["ros_apt_update"],
            onlyif      => $_ros1_bridge,
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

        # If raspberry, set atomic linker flag
        if $raspberry_present == "yes" {
            #$_RPIFLAGS = "-DCMAKE_CXX_FLAGS=-latomic"
            $_RPIFLAGS = "--cmake-args \"-DCMAKE_SHARED_LINKER_FLAGS='-latomic'\" \"-DCMAKE_EXE_LINKER_FLAGS='-latomic'\""
        } else {
            $_RPIFLAGS = ""
        }

        if ! ("install_flag_ros2" in $installflags) {
            file { ["${builddir}", "${builddir}/src", "/etc/ros", "/etc/ros/rosdep"]:
                ensure      => directory,
                owner       => "mav",
                group       => "mav",
                mode        => "755",
            }

            $buildparallel = ceiling((1 + $::processorcount) / 2) # Restrict build parallelization to roughly processors/2
            ensure_packages(["libasio-dev", "libtinyxml2-dev", "libcunit1-dev"])
            install_python_module { ["colcon-common-extensions", "lark-parser", "flake8"]:
                ensure  => present,
                require => Package["libasio-dev"],
            } ->
            exec { "ros2-rosinstall":
                cwd     => "${builddir}",
                command => "/srv/maverick/software/python/bin/rosinstall_generator ros_base --rosdistro ${_distribution} --deps >ros2.repos",
                creates => "${builddir}/ros2.repos",
                timeout => 0,
                user    => "mav",
            } ->
            exec { "ros2-vcs-import":
                cwd     => "${builddir}",
                command => "/srv/maverick/software/python/bin/vcs import src <ros2.repos",
                creates => "${builddir}/src/ros2cli",
                timeout => 0,
                user    => "mav",
            } ->
            exec { "ros2-rosdep-init":
                cwd     => "${builddir}",
                command => "/srv/maverick/software/python/bin/rosdep init",
                creates => "/etc/ros/rosdep/sources.list.d/20-default.list",
                timeout => 0,
                user    => "mav",
            } ->
            exec { "ros2-rosdep-update":
                cwd     => "${builddir}",
                environment     => ["ROS_OS_OVERRIDE=${_osdistro}", "ROS_PYTHON_VERSION=3"],
                command => "/srv/maverick/software/python/bin/rosdep update",
                user    => "mav",
                timeout => 0,
                creates => "/srv/maverick/.ros/rosdep/sources.cache",
            } ->
            exec { "ros2-rosdep-install":
                cwd     => "${builddir}",
                environment     => ["ROS_OS_OVERRIDE=${_osdistro}", "ROS_PYTHON_VERSION=3"],
                command => "/srv/maverick/software/python/bin/rosdep install --from-paths src --ignore-src --rosdistro ${_distribution} ${_osdistro} -y --skip-keys 'console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers ros1_bridge'",
                user    => "mav",
                timeout => 0,
                creates => "${builddir}/src/ros2cli/ros2cli/setup.py"
            } ->
            exec { "ros2-colcon-build":
                cwd     => "${builddir}",
                environment => ["PYTHON_EXECUTABLE=/srv/maverick/software/python/bin/python3", "PATH=/srv/maverick/software/python/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin"],
                command => "/srv/maverick/software/python/bin/colcon build --cmake-force-configure --cmake-args ${_RPIFLAGS} -DCMAKE_VERBOSE_MAKEFILE=ON -DCMAKE_BUILD_TYPE=Release --catkin-skip-building-tests --install-base /srv/maverick/software/ros2/${_distribution}/ --merge-install >/srv/maverick/var/log/build/ros2.colcon.build 2>&1",
                user    => "mav",
                creates => "/srv/maverick/software/ros2/${_distribution}/bin/ros2",
                timeout => 0,
            } ->
            file { "/srv/maverick/var/build/.install_flag_ros2":
                ensure      => present,
            }
            if $_ros1_bridge == true {
                exec { "ros2-rosinstall-ros1_bridge":
                    cwd     => "${builddir}",
                    command => "/srv/maverick/software/python/bin/rosinstall_generator ros1_bridge --rosdistro ${_distribution} >ros1_bridge.repos",
                    creates => "${builddir}/ros1_bridge.repos",
                    user    => "mav",
                    require => Exec["ros2-colcon-build"],
                } ->
                exec { "ros2-vcs-import-ros1_bridge":
                    cwd     => "${builddir}",
                    command => "/srv/maverick/software/python/bin/vcs import src <ros1_bridge.repos",
                    creates => "${builddir}/src/ros1_bridge",
                    user    => "mav",
                } ->
                exec { "ros2-colcon-build-ros1_bridge":
                    cwd     => "${builddir}",
                    environment => ["PYTHON_EXECUTABLE=/srv/maverick/software/python/bin/python3", "PATH=/srv/maverick/software/python/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"],
                    command => "/bin/bash --login -c 'source /srv/maverick/software/ros/current/setup.bash; source /srv/maverick/software/ros2/${_distribution}/local_setup.bash; /srv/maverick/software/python/bin/colcon build --cmake-force-configure --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON -DBUILD_TESTING=0 -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/srv/maverick/software/python/bin/python3 --catkin-skip-building-tests --install-base /srv/maverick/software/ros2/${_distribution} --merge-install --packages-select ros1_bridge >/srv/maverick/var/log/build/ros2.colcon.ros1_bridge.build 2>&1'",
                    creates => "/srv/maverick/software/ros2/${_distribution}/ros1_bridge/lib/libros1_bridge.so",
                    user    => "mav",
                    timeout => 0,
                    before  => File["/srv/maverick/var/build/.install_flag_ros2"],
                }
            }
        }
    }

}
