class maverick_ros::ros2 (
    $installtype = "auto",
    $distribution = "auto",
    $installdir = "/srv/maverick/software/ros2",
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
                        $autodist = "ardent"
                        case $architecture {
                            "amd64", "arm64", "aarch64": { $_installtype = "native" }
                            default: { $_installtype = "source" }
                        }
                    }
                    "yakkety": {
                        $autodist = "ardent"
                        $_installtype = "source"
                    }
                    "zesty": {
                        $autodist = "ardent"
                        $_installtype = "source"
                    }
                    "bionic": {
                        $autodist = "ardent"
                        $_installtype = "source"
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
                        $autodist = "ardent"
                        $_installtype = "source"
                    }
                    "9": { # stretch
                        $autodist = "ardent"
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
    
    if $installtype == "native" and $_distribution {
        # Install dependencies
        
        # Work out distro name
        if $::lsbdistid == "ubilinux" and $::lsbmajdistrelease == "4" {
            $_distro = "stretch"
        } else {
            $_distro = $::lsbdistcodename
        }
        
        # Create symlink to usual vendor install directory
        file { ["/opt/ros2"]:
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
        file { "/opt/ros2/${_distribution}":
            ensure      => link,
            target      => "${installdir}/${_distribution}",
            force       => true,
        } ->
        file { "${installdir}/current":
            ensure      => link,
            target      => "${installdir}/${_distribution}",
            force       => true,
        } ->
        # Install ROS2 repo/key
        exec { "ros2-repo-key":
            command     => "/usr/bin/curl http://repo.ros2.org/repos.key | apt-key add -",
            unless      => "/usr/bin/apt-key list |/bin/egrep 'B01F\s?A116'",
        } ->
        exec { "ros2-repo":
            command     => "/bin/echo \"deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main ${_distro} main\" > /etc/apt/sources.list.d/ros2-latest.list",
            unless      => "/bin/grep '${_distro}' /etc/apt/sources.list.d/ros2-latest.list",
            notify      => Exec["apt_update"],
        }
        
    }
}