class maverick_ros::ros2 (
    $installtype = "auto",
    $distribution = "auto",
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
            before      => Package["ros-${_distribution}-${metapackage}"]
        }
    }

    # Install from ros repos
    if $_installtype == "native" {
        package { "ros-${_distribution}-${metapackage}":
            ensure      => present,
        } ->
        package { "ros-${_distribution}-ros1-bridge":
            ensure      => present,
        } ->
        package { ["ros-${_distribution}-cv-bridge", "ros-${_distribution}-vision-opencv", "ros-${_distribution}-image-geometry"]:
            ensure      => present,
        }
    }

}
