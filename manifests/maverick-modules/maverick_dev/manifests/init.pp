class maverick_dev (
    $sitl = true,
    $ardupilot = true,
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

    if $sitl {
        class { "maverick_dev::sitl": }
    }

    class { "maverick_dev::dronekit": }

}