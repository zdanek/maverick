# @summary
#   Maverick_ros::Ros1 class
#   This class installs/manages ROS (ros.org).
#
#   For Raspbery OS starting from Bullseye, ROS 1 is not supported anymore.
#
# @example Declaring the class
#   This class is included from maverick_ros class and should not be included from elsewhere
#
# @param installtype
#   If 'auto' then the type of install needed will be detected based on the operating system version and architecture.  This can be overidden with 'native' or 'source'.
# @param distribution
#   Specify the ROS distribution to use - eg. melodic, noetic
# @param buildtype
#   ROS core variant, robot is comprehensive base without GUI, could also be desktop, desktop_full etc.  See https://www.ros.org/reps/rep-0131.html#variants.
# @param builddir
#   Set the build location.
# @param installdir
#   Set the install location, should always be /srv/maverick/sofware/ros (which is symlinked to /opt/ros) in Maverick
# @param module_mavros
#   If true, install mavros
# @param module_realsense
#   If true, install realsense support
# @param module_opencv
#   If true, install opencv related ros modules
#
class maverick_ros::ros1 (
    Enum['native', 'source', 'auto'] $installtype = "auto",
    String $distribution = "auto",
    String $buildtype = "robot",
    String $builddir = "/srv/maverick/var/build/ros_catkin_ws",
    String $installdir = "/srv/maverick/software/ros",
    Boolean $module_mavros = true,
    Boolean $module_realsense = false,
    Boolean $module_opencv = false,
    Boolean $module_apriltag = false,
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
                    "focal": {
                        $autodist = "noetic"
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
            "Debian", "Raspbian": {
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
                            default: { $autodist = "melodic" }
                        }
                        case $::architecture {
                            "amd64", "arm64": { $_installtype = "native" }
                            default: { $_installtype = "source" }
                        }
                    }
                    "10": { # buster
                        case $::raspberry_present {
                            "yes": { $autodist = "noetic" }
                            default: { $autodist = "noetic" }
                        }
                        case $::architecture {
                            "amd64", "arm64": { $_installtype = "native" }
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
        $_installtype = undef
    }

    if !$_installtype {
        # fail("ROS: Cannot decide which ROS install, on ${::operatingsystem}, ${::operatingsystemmajrelease}, ${::architecture}, Install type: ${installtype}, Distribution: ${distribution}, Build type: ${buildtype}")
        warning("ROS 1 is not supported for ${::operatingsystem}, ${::operatingsystemmajrelease}, ${::architecture}. Skipping.")
        return()
    }

    if $distribution == "auto" and $installtype == "auto" {
        $_distribution = $autodist
    } else {
        $_distribution = $distribution
    }

    if $installtype and $_distribution {
        # Install dependencies
        ensure_packages(["gnupg"])
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
            require     => Package["gnupg"],
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
        package { ["python-rosdep", "python3-rosdep", "python-catkin-tools", "python3-catkin-tools", "python-wstool", "python3-wstool", "python-vcstools", "python3-vcstools"]:
            ensure      => absent,
        } ->
        install_python_module { ["wstool", "trollius", "rospkg", "catkin-pkg", "rosinstall", "sip", "osrf-pycommon"]:
            ensure      => present,
        } ->
        install_python_module { "catkin-tools":
            pkgname     => "git+https://github.com/catkin/catkin_tools.git#egg=catkin_tools",
            ensure      => present,
        } ->
        file { "/etc/profile.d/30-ros1env.sh":
            ensure      => present,
            mode        => "644",
            owner       => "root",
            group       => "root",
            content     => "ros1env() { if [ -f /srv/maverick/config/ros/rosmaster-\$1.conf ]; then . /srv/maverick/config/ros/rosmaster-\$1.conf; else echo \"Error: ROS1 config for \$1 instance does not exist\"; fi }",
        } ->
        file { "/etc/profile.d/30-ros-env.sh":
            ensure      => absent,
        } ->
        file { "/etc/profile.d/31-ros-env.sh":
            ensure      => present,
            mode        => "644",
            owner       => "root",
            group       => "root",
            content     => "source /opt/ros/${_distribution}/setup.bash\nros1env fc",
        }
    }

    # Install from ros repos
    if $_installtype == "native" {
        package { ["ros-${_distribution}-perception", "ros-${_distribution}-viz", "ros-${_distribution}-mavros", "ros-${_distribution}-mavros-extras", "ros-${_distribution}-mavros-msgs", "ros-${_distribution}-test-mavros"]:
            ensure      => installed,
            require     => [ Exec["ros_apt_update"], Install_python_module["rosdep"], File["/opt/ros/${_distribution}"], ],
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
            command         => "/srv/maverick/software/python/bin/rosdep init",
            creates         => "/etc/ros/rosdep/sources.list.d/20-default.list",
        } ->
        exec { "rosdep-update":
            user            => "mav",
            command         => "/srv/maverick/software/python/bin/rosdep update",
            creates         => "/srv/maverick/.ros/rosdep/sources.cache",
        } ->
        exec { "ros-install-marker":
            command     => "/bin/false",
            refreshonly => true,
        }
        if $module_apriltag == true {
            package { "ros-${_distribution}-apriltag-ros":
                ensure  => present,
                require     => Exec["ros_apt_update"],
            }
        }

    # Build from source
    } elsif $_installtype == "source" {

        # Force osdistro for raspberry
        if $raspberry_present == "yes" {
            #$_osdistro = "--os=debian:${::lsbdistcodename}"
            $_osdistro = "debian:${::lsbdistcodename}"
        } else {
            $_osdistro = ""
        }

        if ! ("install_flag_ros" in $installflags) {
            file { ["${builddir}", "${builddir}/src"]:
                ensure      => directory,
                owner       => "mav",
                group       => "mav",
                mode        => "755",
            }
            $buildparallel = ceiling((1 + $::processorcount) / 2) # Restrict build parallelization to roughly processors/2 if raspberry
            # Initialize rosdep
            exec { "rosdep-init":
                path            => ["/srv/maverick/software/python/bin", "/usr/bin", "/bin"],
                command         => "rosdep init",
                creates         => "/etc/ros/rosdep/sources.list.d/20-default.list",
                require         => [ Install_python_module["rosdep"], Install_python_module["wstool"], Install_python_module["rosinstall"], Install_python_module["rosinstall-generator"] ],
            } ->
            exec { "rosdep-update":
                path            => ["/srv/maverick/software/python/bin", "/usr/bin", "/bin"],
                environment     => ["ROS_OS_OVERRIDE=${_osdistro}", "ROS_PYTHON_VERSION=3"],
                user            => "mav",
                timeout         => 0,
                command         => "rosdep update --rosdistro ${_distribution} -y >/srv/maverick/var/log/build/ros.rosdep_update.out 2>&1",
                creates         => "/srv/maverick/.ros/rosdep/sources.cache",
                cwd             => "${builddir}",
            } ->
            exec { "catkin_rosinstall":
                path            => ["/srv/maverick/software/python/bin", "/usr/bin", "/bin"],
                command         => "rosinstall_generator ${buildtype} --rosdistro ${_distribution} --deps --tar --wet-only > ${_distribution}-${buildtype}.rosinstall",
                cwd             => "${builddir}",
                user            => "mav",
                creates         => "${builddir}/${_distribution}-${buildtype}.rosinstall",
            } ->
            exec { "catkin_vcsimport":
                path            => ["/srv/maverick/software/python/bin", "/usr/bin", "/bin"],
                command         => "vcs import --input ${_distribution}-${buildtype}.rosinstall ./src >/srv/maverick/var/log/build/ros.vscimport.out 2>&1",
                cwd             => "${builddir}",
                user            => "mav",
                creates         => "${builddir}/src/catkin",
            } ->
            exec { "catkin_rosdep":
                path            => ["/srv/maverick/software/python/bin", "/usr/bin", "/bin"],
                command         => "rosdep install --from-paths src --ignore-packages-from-source --os=${_osdistro} --rosdistro ${_distribution} -y >/srv/maverick/var/log/build/ros.rosdep_install.out 2>&1",
                environment     => ["ROS_OS_OVERRIDE=${_osdistro}", "ROS_PYTHON_VERSION=3"],
                cwd             => "${builddir}",
                user            => "mav",
                timeout         => 0,
            } ->
            exec { "catkin_make":
                command         => "${builddir}/src/catkin/bin/catkin_make_isolated --install --install-space ${installdir}/${_distribution} -DPYTHON_EXECUTABLE=/srv/maverick/software/python/bin/python3 -DSETUPTOOLS_DEB_LAYOUT=OFF -DCMAKE_BUILD_TYPE=Release -j${buildparallel} >/srv/maverick/var/log/build/ros.catkin_make.out 2>&1",
                cwd             => "${builddir}",
                environment     => ["ROS_OS_OVERRIDE=${_osdistro}", "ROS_PYTHON_VERSION=3"],
                user            => "mav",
                creates         => "${installdir}/${_distribution}/lib/rosbag/topic_renamer.py",
                timeout         => 0,
                require         => File["${installdir}/${_distribution}"],
            } ->
            file { "/srv/maverick/var/build/.install_flag_ros":
                ensure      => present,
            } ->
            exec { "ros-install-marker":
                command     => "/bin/false",
                refreshonly => true,
            }
        } else {
            exec { "ros-install-marker":
                command     => "/bin/false",
                refreshonly => true,
            }
        }

        if $module_mavros == true {
            if ! ("install_flag_ros_mavros" in $installflags) {
                package { ["liburdfdom-dev", "liburdfdom-headers-dev", "libtf2-bullet-dev", "libbondcpp-dev", "geographiclib-tools", "libgeographic-dev"]:
                    ensure => installed,
                } ->
                file { ["/srv/maverick/var/build/catkin_ws_mavros", "/srv/maverick/var/build/catkin_ws_mavros/src"]:
                    ensure  => directory,
                    owner   => "mav",
                } ->
                exec { "ros-mavros-catkin-init":
                    user        => "mav",
                    cwd         => "/srv/maverick/var/build/catkin_ws_mavros",
                    path        => ["/srv/maverick/software/python/bin", "/usr/bin", "/bin"],
                    command     => "catkin config --init --extend /opt/ros/current --install -i /opt/ros/current",
                    environment => ["PYTHONPATH=/opt/ros/current/lib/python3/dist-packages:/srv/maverick/software/python/lib/python3.8/site-packages:/srv/maverick/software/python/lib/python3.8/site-packages", "CMAKE_PREFIX_PATH=/opt/ros/${_distribution}"],
                    creates     => "/srv/maverick/var/build/catkin_ws_mavros/.catkin_tools/profiles/default/config.yaml",
                    require     => [ Install_python_module["catkin-tools"], Install_python_module["rosinstall-generator"], Exec["ros-install-marker"], ],
                } ->
                exec { "ros-mavros-rosinstall_mavlink":
                    user        => "mav",
                    cwd         => "/srv/maverick/var/build/catkin_ws_mavros",
                    path        => ["/srv/maverick/software/python/bin", "/usr/bin", "/bin"],
                    command     => "rosinstall_generator --rosdistro ${_distribution} mavlink uuid_msgs geographic_msgs control_toolbox realtime_tools tf2_eigen >/srv/maverick/var/build/catkin_ws_mavros/mavros.rosinstall",
                    creates     => "/srv/maverick/var/build/catkin_ws_mavros/mavros.rosinstall",
                } ->
                exec { "ros-mavros-rosinstall_mavros":
                    user        => "mav",
                    cwd         => "/srv/maverick/var/build/catkin_ws_mavros",
                    path        => ["/srv/maverick/software/python/bin", "/usr/bin", "/bin"],
                    command     => "rosinstall_generator --rosdistro ${_distribution} --upstream mavros mavros_extras mavros_msgs >>/srv/maverick/var/build/catkin_ws_mavros/mavros.rosinstall",
                    unless      => "/bin/grep mavros mavros.rosinstall",
                } ->
                exec { "ros-mavros_vcsimport":
                    path            => ["/srv/maverick/software/python/bin", "/usr/bin", "/bin"],
                    command         => "vcs import --input mavros.rosinstall ./src >/srv/maverick/var/log/build/mavros.vscimport.out 2>&1",
                    cwd             => "/srv/maverick/var/build/catkin_ws_mavros",
                    user            => "mav",
                    creates         => "${builddir}/src/mavros",
                } ->
                # Install mavros geographiclib dependencies
                exec { "mavros_geoinstall":
                    command     => "/bin/bash /srv/maverick/var/build/catkin_ws_mavros/src/mavros/mavros/scripts/install_geographiclib_datasets.sh >/srv/maverick/var/log/build/ros.mavros_geoinstall.out 2>&1",
                    creates     => "/usr/share/GeographicLib/geoids/egm96-5.pgm",
                    require     => Package["geographiclib-tools"],
                } ->
                /*
                exec { "ros-mavros-rosdep-install":
                    user        => "mav",
                    environment => ["CMAKE_PREFIX_PATH=/opt/ros/current", "PYTHONPATH=/srv/maverick/software/ros/${_distribution}/lib/python3.8/site-packages:/srv/maverick/software/ros/${_distribution}/lib/python3/dist-packages:/srv/maverick/software/python/lib/python3.8/site-packages"],
                    cwd         => "/srv/maverick/var/build/catkin_ws_mavros",
                    path        => ["/srv/maverick/software/python/bin", "/usr/bin", "/bin"],
                    command     => "rosdep install --from-paths src --ignore-src --rosdistro ${_distribution} -y ${_osdistro} -r >/srv/maverick/var/log/build/mavros.rosdep.out 2>&1",
                } ->
                */
                exec { "ros-mavros-catkin-build":
                    user        => "mav",
                    cwd         => "/srv/maverick/var/build/catkin_ws_mavros",
                    environment => ["CMAKE_PREFIX_PATH=/opt/ros/current", "PYTHONPATH=/srv/maverick/software/ros/${_distribution}/lib/python3.8/site-packages:/srv/maverick/software/ros/${_distribution}/lib/python3/dist-packages:/srv/maverick/software/python/lib/python3.8/site-packages"],
                    path        => ["/srv/maverick/software/python/bin", "/usr/bin", "/bin"],
                    command     => "catkin build -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release -DSETUPTOOLS_DEB_LAYOUT=OFF -j2 -l2 >/srv/maverick/var/log/build/mavros-catkin-build.out 2>&1",
                    creates     => "/srv/maverick/ros/current/share/mavros",
                    timeout     => 0,
                } ->
                file { "/srv/maverick/var/build/.install_flag_ros_mavros":
                    ensure      => present,
                }
            }
        }
        if $module_apriltag == true {
            if ! ("install_flag_ros_apriltag" in $installflags) {
                file { ["/srv/maverick/var/build/catkin_ws_apriltag", "/srv/maverick/var/build/catkin_ws_apriltag/src"]:
                    ensure  => directory,
                    owner   => "mav",
                } ->
                exec { "ros-apriltag-catkin-init":
                    user        => "mav",
                    cwd         => "/srv/maverick/var/build/catkin_ws_apriltag",
                    path        => ["/srv/maverick/software/python/bin", "/usr/bin", "/bin"],
                    command     => "catkin config --init --extend /opt/ros/current --install -i /opt/ros/current",
                    environment => ["PYTHONPATH=/srv/maverick/software/ros/${_distribution}/lib/python3.8/site-packages:/srv/maverick/software/ros/${_distribution}/lib/python3/dist-packages:/srv/maverick/software/python/lib/python3.8/site-packages", "CMAKE_PREFIX_PATH=/opt/ros/${_distribution}:/srv/maverick/software/opencv:/srv/maverick/software/apriltag"],
                    creates     => "/srv/maverick/var/build/catkin_ws_apriltag/.catkin_tools/profiles/default/config.yaml",
                    require     => [ Install_python_module["catkin-tools"], Install_python_module["rosinstall-generator"], Exec["ros-install-marker"], ],
                } ->
                exec { "ros-rosinstall_apriltag":
                    user        => "mav",
                    cwd         => "/srv/maverick/var/build/catkin_ws_apriltag",
                    path        => ["/srv/maverick/software/python/bin", "/usr/bin", "/bin"],
                    environment => ["PYTHONPATH=/srv/maverick/software/ros/${_distribution}/lib/python3.8/site-packages:/srv/maverick/software/ros/${_distribution}/lib/python3/dist-packages:/srv/maverick/software/python/lib/python3.8/site-packages", "CMAKE_PREFIX_PATH=/opt/ros/${_distribution}:/srv/maverick/software/opencv:/srv/maverick/software/apriltag"],
                    command     => "rosinstall_generator --rosdistro ${_distribution} image_transport nodelet bond bondcpp smclib >/srv/maverick/var/build/catkin_ws_apriltag/apriltag.rosinstall",
                    creates     => "/srv/maverick/var/build/catkin_ws_apriltag/apriltag.rosinstall",
                } ->
                oncevcsrepo { "git-ros-apriltag":
                    gitsource   => "https://github.com/AprilRobotics/apriltag.git",
                    dest        => "/srv/maverick/var/build/catkin_ws_apriltag/src/apriltag",
                } ->
                oncevcsrepo { "git-ros-apriltag_ros":
                    gitsource   => "https://github.com/AprilRobotics/apriltag_ros.git",
                    dest        => "/srv/maverick/var/build/catkin_ws_apriltag/src/apriltag_ros",
                } ->
                exec { "ros-apriltag_vcsimport":
                    path            => ["/srv/maverick/software/python/bin", "/usr/bin", "/bin"],
                    command         => "vcs import --input apriltag.rosinstall ./src >/srv/maverick/var/log/build/apriltag.vscimport.out 2>&1",
                    cwd             => "/srv/maverick/var/build/catkin_ws_apriltag",
                    user            => "mav",
                    creates         => "${builddir}/src/apriltag",
                } ->
                exec { "ros-apriltag-catkin-build":
                    user        => "mav",
                    cwd         => "/srv/maverick/var/build/catkin_ws_apriltag",
                    path        => ["/srv/maverick/software/python/bin", "/usr/bin", "/bin"],
                    environment => ["LDFLAGS=-Wl,-rpath,/srv/maverick/software/python/lib", "LD_LIBRARY_PATH=/srv/maverick/software/python/lib", "PYTHONPATH=/srv/maverick/software/ros/${_distribution}/lib/python3.8/site-packages:/srv/maverick/software/ros/${_distribution}/lib/python3/dist-packages:/srv/maverick/software/python/lib/python3.8/site-packages", "CMAKE_PREFIX_PATH=/opt/ros/${_distribution}:/srv/maverick/software/opencv:/srv/maverick/software/apriltag"],
                    command     => "catkin build -DPYTHON_EXECUTABLE=/srv/maverick/software/python/bin/python3 -DPYTHON_INCLUDE_DIR=/srv/maverick/software/python/include/python3.8 -DPYTHON_LIBRARY=/srv/maverick/software/python/lib/libpython3.8.so -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release -DSETUPTOOLS_DEB_LAYOUT=OFF -j2 -l2 >/srv/maverick/var/log/build/apriltag-catkin-build.out 2>&1",
                    creates     => "/opt/ros/current/share/apriltag_ros",
                    timeout     => 0,
                } ->
                file { "/srv/maverick/var/build/.install_flag_ros_apriltag":
                    ensure      => present,
                }
            }
        }

    }

    # Install vision_opencv and cv_bridge, linked to custom opencv
    if $module_opencv == true {
        if ! ("install_flag_ros_opencv" in $installflags) {
            file { ["/srv/maverick/var/build/catkin_ws_opencv", "/srv/maverick/var/build/catkin_ws_opencv/src"]:
                ensure  => directory,
                owner   => "mav",
            } ->
            install_python_module { "empy":
                version     => "3.3.4",
                ensure      => present,
            } ->
            # Install vision_opencv from a forked branch, fixed for opencv4 - see https://github.com/ros-perception/vision_opencv/issues/320
            oncevcsrepo { "git-ros-vision_opencv":
                #gitsource   => "https://github.com/fnoop/vision_opencv",
                gitsource   => "https://github.com/ros-perception/vision_opencv.git",
                revision    => "melodic",
                dest        => "/srv/maverick/var/build/catkin_ws_opencv/src/vision_opencv",
                depth       => undef,
            } ->
            exec { "ros-opencv-init-workspace":
                user        => "mav",
                cwd         => "/srv/maverick/var/build/catkin_ws_opencv/src",
                command     => "/opt/ros/current/bin/catkin_init_workspace",
                environment => ["PYTHONPATH=/srv/maverick/software/ros/${_distribution}/lib/python3.8/site-packages:/srv/maverick/software/ros/${_distribution}/lib/python3/dist-packages:/srv/maverick/software/ros/${_distribution}/lib/python2.7/dist-packages:/srv/maverick/software/python/lib/python3.8/site-packages", "CMAKE_PREFIX_PATH=/opt/ros/${_distribution}:/srv/maverick/software/opencv", "CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-10.2"],
                creates     => "/srv/maverick/var/build/catkin_ws_opencv/src/CMakeLists.txt",
                require     => [ Install_python_module["catkin-tools"], Exec["ros-install-marker"], ],
            } ->
            exec { "ros-opencv-catkin-make":
                user        => "mav",
                cwd         => "/srv/maverick/var/build/catkin_ws_opencv",
                command     => "/srv/maverick/software/python/bin/python3 /opt/ros/current/bin/catkin_make -DPYTHON_EXECUTABLE=/srv/maverick/software/python/bin/python3 -DPYTHON_INCLUDE_DIR=/srv/maverick/software/python/include/python3.8 -DPYTHON_LIBRARY=/srv/maverick/software/python/lib/libpython3.so clean; /srv/maverick/software/python/bin/python3 /opt/ros/current/bin/catkin_make -DPYTHON_EXECUTABLE=/srv/maverick/software/python/bin/python3 -DPYTHON_LIBRARY=/srv/maverick/software/python/lib/libpython3.so -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-10.2 -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=CMAKE_PREFIX_PATH=/opt/ros/${_distribution}:/srv/maverick/software/opencv -DSETUPTOOLS_DEB_LAYOUT=OFF -DCMAKE_INSTALL_PREFIX=${installdir}/${_distribution} >/srv/maverick/var/log/build/ros.opencv.catkin_make.out 2>&1",
                timeout     => 0,
                environment => ["PYTHON=/srv/maverick/software/python/bin/python3", "PATH=/srv/maverick/software/python/bin:/usr/bin:/bin", "PYTHONPATH=/srv/maverick/software/ros/${_distribution}/lib/python3.8/site-packages:/srv/maverick/software/ros/${_distribution}/lib/python3/dist-packages:/srv/maverick/software/ros/${_distribution}/lib/python2.7/dist-packages:/srv/maverick/software/python/lib/python3.8/site-packages", "CMAKE_PREFIX_PATH=/opt/ros/${_distribution}:/srv/maverick/software/opencv:/srv/maverick/software/realsense-sdk2"],
                require     => Class["maverick_vision::opencv"],
                before      => Exec["ros-realsense-catkin-make"],
            } ->
            exec { "ros-opencv-catkin-install":
                user        => "root",
                cwd         => "/srv/maverick/var/build/catkin_ws_opencv",
                command     => "/opt/ros/current/bin/catkin_make install -DPYTHON_EXECUTABLE=/srv/maverick/software/python/bin/python3 -DPYTHON_INCLUDE_DIR=/srv/maverick/software/python/include/python3.8 -DPYTHON_LIBRARY=/srv/maverick/software/python/lib/libpython3.so -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-10.2 -DSETUPTOOLS_DEB_LAYOUT=OFF -DCMAKE_INSTALL_PREFIX=${installdir}/${_distribution} >/srv/maverick/var/log/build/ros.opencv.catkin_install.out 2>&1",
                timeout     => 0,
                environment => ["PYTHON=/srv/maverick/software/python/bin/python3", "PATH=/srv/maverick/software/python/bin:/usr/bin:/bin", "PYTHONPATH=/srv/maverick/software/ros/${_distribution}/lib/python3.8/site-packages:/srv/maverick/software/ros/${_distribution}/lib/python3/dist-packages:/srv/maverick/software/ros/${_distribution}/lib/python2.7/dist-packages:/srv/maverick/software/python/lib/python3.8/site-packages", "CMAKE_PREFIX_PATH=/opt/ros/${_distribution}:/srv/maverick/software/opencv:/srv/maverick/software/realsense-sdk2"],
            } ->
            file { "/srv/maverick/var/build/.install_flag_ros_opencv":
                ensure      => present,
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
            oncevcsrepo { "git-ros-realsense":
                gitsource   => "https://github.com/IntelRealSense/realsense-ros.git",
                dest        => "/srv/maverick/var/build/catkin_ws_realsense/src/realsense-ros",
                revision    => "2.2.15",
                depth       => undef,
            } ->
            exec { "ros-realsense-init-workspace":
                user        => "mav",
                cwd         => "/srv/maverick/var/build/catkin_ws_realsense/src",
                command     => "/opt/ros/current/bin/catkin_init_workspace",
                environment => ["PYTHONPATH=/srv/maverick/software/ros/${_distribution}/lib/python3.8/site-packages:/srv/maverick/software/ros/${_distribution}/lib/python3/dist-packages:/srv/maverick/software/ros/${_distribution}/lib/python2.7/dist-packages:/srv/maverick/software/python/lib/python3.8/site-packages", "CMAKE_PREFIX_PATH=/opt/ros/${_distribution}:/srv/maverick/software/opencv:/srv/maverick/software/realsense-sdk2"],
                creates     => "/srv/maverick/var/build/catkin_ws_realsense/src/CMakeLists.txt",
                require     => [ Install_python_module["catkin-tools"], Exec["ros-install-marker"], Class["maverick_hardware::peripheral::realsense"]],
            } ->
            exec { "ros-rosinstall_realsense":
                user        => "mav",
                cwd         => "/srv/maverick/var/build/catkin_ws_realsense",
                path        => ["/srv/maverick/software/python/bin", "/usr/bin", "/bin"],
                environment => ["PYTHONPATH=/srv/maverick/software/ros/${_distribution}/lib/python3.8/site-packages:/srv/maverick/software/ros/${_distribution}/lib/python3/dist-packages:/srv/maverick/software/ros/${_distribution}/lib/python2.7/dist-packages:/srv/maverick/software/python/lib/python3.8/site-packages", "CMAKE_PREFIX_PATH=/opt/ros/${_distribution}:/srv/maverick/software/opencv:/srv/maverick/software/realsense-sdk2"],
                command     => "rosinstall_generator --rosdistro ${_distribution} ddynamic_reconfigure >/srv/maverick/var/build/catkin_ws_realsense/realsense.rosinstall",
                creates     => "/srv/maverick/var/build/catkin_ws_realsense/realsense.rosinstall",
            } ->
            exec { "ros-realsense_vcsimport":
                path            => ["/srv/maverick/software/python/bin", "/usr/bin", "/bin"],
                command         => "vcs import --input realsense.rosinstall ./src >/srv/maverick/var/log/build/realsense.vscimport.out 2>&1",
                cwd             => "/srv/maverick/var/build/catkin_ws_realsense",
                user            => "mav",
                creates         => "/srv/maverick/var/build/catkin_ws_realsense/src/ddynamic_reconfigure",
            } ->
            exec { "ros-realsense-catkin-make":
                user        => "mav",
                cwd         => "/srv/maverick/var/build/catkin_ws_realsense",
                command     => "/srv/maverick/software/python/bin/python3 /opt/ros/current/bin/catkin_make -DPYTHON_EXECUTABLE=/srv/maverick/software/python/bin/python3 -DPYTHON_LIBRARY=/srv/maverick/software/python/lib/libpython3.so clean; /srv/maverick/software/python/bin/python3 /opt/ros/current/bin/catkin_make -DPYTHON_EXECUTABLE=/srv/maverick/software/python/bin/python3 -DPYTHON_LIBRARY=/srv/maverick/software/python/lib/libpython3.so -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${installdir}/${_distribution} >/srv/maverick/var/log/build/ros.realsense.catkin_make.out 2>&1",
                timeout     => 0,
                environment => ["PYTHON=/srv/maverick/software/python/bin/python3", "PATH=/srv/maverick/software/python/bin:/usr/bin:/bin", "PYTHONPATH=/srv/maverick/software/ros/${_distribution}/lib/python3.8/site-packages:/srv/maverick/software/ros/${_distribution}/lib/python3/dist-packages:/srv/maverick/software/ros/${_distribution}/lib/python2.7/dist-packages:/srv/maverick/software/python/lib/python3.8/site-packages", "CMAKE_PREFIX_PATH=/opt/ros/${_distribution}:/srv/maverick/software/opencv:/srv/maverick/software/realsense-sdk2"],
                creates     => "/srv/maverick/var/build/catkin_ws_realsense/build/realsense-ros/realsense2_camera/catkin_generated/installspace/realsense2_camera.pc",
            } ->
            exec { "ros-realsense-catkin-install":
                user        => "root",
                cwd         => "/srv/maverick/var/build/catkin_ws_realsense",
                command     => "/srv/maverick/software/python/bin/python3 /opt/ros/current/bin/catkin_make install -DPYTHON_EXECUTABLE=/srv/maverick/software/python/bin/python3 -DPYTHON_LIBRARY=/srv/maverick/software/python/lib/libpython3.so -DCMAKE_INSTALL_PREFIX=${installdir}/${_distribution} >/srv/maverick/var/log/build/ros.realsense.catkin_install.out 2>&1",
                timeout     => 0,
                environment => ["PYTHON=/srv/maverick/software/python/bin/python3", "PATH=/srv/maverick/software/python/bin:/usr/bin:/bin", "PYTHONPATH=/srv/maverick/software/ros/${_distribution}/lib/python3.8/site-packages:/srv/maverick/software/ros/${_distribution}/lib/python3/dist-packages:/srv/maverick/software/ros/${_distribution}/lib/python2.7/dist-packages:/srv/maverick/software/python/lib/python3.8/site-packages", "CMAKE_PREFIX_PATH=/opt/ros/${_distribution}:/srv/maverick/software/opencv:/srv/maverick/software/realsense-sdk2"],
                creates     => "/srv/maverick/software/ros/current/lib/librealsense2_camera.so",
            } ->
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
        file { "/etc/profile.d/31-maverick-ros-pythonpath.sh":
            mode        => "644",
            owner       => "root",
            group       => "root",
            content     => 'NEWPATH="/srv/maverick/software/ros/current/lib/python2.7/dist-packages:/srv/maverick/software/ros/current/lib/python3/dist-packages:/srv/maverick/software/ros/current/lib/python3.8/site-packages"; export PYTHONPATH=${PYTHONPATH:-${NEWPATH}}; if [ -n "${PYTHONPATH##*${NEWPATH}}" -a -n "${PYTHONPATH##*${NEWPATH}:*}" ]; then export PYTHONPATH=$NEWPATH:$PYTHONPATH; fi',
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
}
