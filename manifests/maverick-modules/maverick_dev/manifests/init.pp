class maverick_dev (
    $sitl = true,
    $ardupilot = true,
) {
   
    if $ardupilot {
        class { "maverick_dev::ardupilot": 
            sitl    => $sitl,
        }
    }
    
    if $sitl {
        class { "maverick_dev::sitl": }
    }
    
    class { "maverick_dev::dronekit": }

}