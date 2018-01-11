class maverick_dev (
    $sitl = true,
    $ardupilot = true,
    $px4 = false,
) {
   
    file { ["/srv/maverick/data/dev", "/srv/maverick/config/dev"]:
        ensure      => directory,
        mode        => "755",
        owner       => "mav",
        group       => "mav",
    }

    if $ardupilot {
        class { "maverick_dev::ardupilot": 
            sitl    => $sitl,
        }
    }
    
    if $px4 == true {
        class { "maverick_dev::px4":
            sitl    => $sitl,
        }
    }

    if $sitl {
        class { "maverick_dev::sitl": }
    }

    class { "maverick_dev::dronekit": }

}