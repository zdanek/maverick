class maverick_ros (
    $ros1 = true,
    $ros2 = true,
) {

    # Install ros gpg key - used for both ros1 and ros2
    apt::key { 'ros-repo-key':
        id      => 'C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654',
        server  => 'keyserver.ubuntu.com',
        require     => Package["dirmngr"],
    }

    if $ros1 == true {
        class { "maverick_ros::ros1": }
    }

    if $ros2 == true {
        class { "maverick_ros::ros2": }
    }
}
