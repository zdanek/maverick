class maverick_ros (
    $ros1 = true,
    $ros2 = true,
) {
    if $ros1 == true {
        class { "maverick_ros::ros1": }
    }

    if $ros2 == true {
        class { "maverick_ros::ros2": }
    }
}
