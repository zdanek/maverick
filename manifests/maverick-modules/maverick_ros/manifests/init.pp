# @summary
#   Maverick_ros class
#   This class controls all other classes in maverick_ros module.
#
#   On latest Raspberry Pi OS, ROS1 is not supported, so ros1 is disabled by default. This applies to Ros master as
#   this is the only supported platform for ros1.
#
# @example Declaring the class
#   This class is included from the environment manifests and is not usually included elsewhere.
#   It could be included selectively from eg. minimal environment.
#
# @param ros1
#   If true, include the maverick_ros::ros1 class which manages ROS1.
# @param ros2
#   If true, include the maverick_ros::ros2 class which manages ROS2.
#
class maverick_ros (
    Boolean $ros1 = true,
    Boolean $ros2 = true,
) {

    # Install ros gpg key - used for both ros1 and ros2
    apt::key { 'ros-repo-key':
        id      => 'C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654',
        server  => 'keyserver.ubuntu.com',
        require     => Package["gnupg"],
    }

    # Install common python modules to ros1 and ros2
    install_python_module { ["rosdep", "vcstool", "rosinstall-generator"]:
        ensure  => present,
    }

    if $ros1 == true {
        class { "maverick_ros::ros1": }
    }

    if $ros2 == true {
        class { "maverick_ros::ros2": }
    }
}
