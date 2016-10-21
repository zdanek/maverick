class maverick_ros (
    $ros_installtype = "",
    $mavros_install = true
) {

    class { "maverick_ros::ros":
        installtype => $ros_installtype
    }

    if $mavros_install == true {
        #class { "maverick_ros::mavros": }
    }
    
}