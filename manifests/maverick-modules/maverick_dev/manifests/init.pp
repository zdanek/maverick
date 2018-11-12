class maverick_dev (
    $apsitl_dev = true,
    $ardupilot = true,
    $px4 = false,
    $px4sitl_dev = true
) {
   
    # Create various dev directories
    file { ["/srv/maverick/data/dev", "/srv/maverick/data/dev/mavlink", "/srv/maverick/config/dev", "/srv/maverick/var/log/dev", "/srv/maverick/var/log/dev/mavlink"]:
        ensure      => directory,
        mode        => "755",
        owner       => "mav",
        group       => "mav",
    }

    # Install files needed to service sitl instances
    file { "/etc/systemd/system/maverick-apsitl@.service":
        source      => "puppet:///modules/maverick_dev/maverick-apsitl@.service",
        owner       => "root",
        group       => "root",
        mode        => "644",
        notify      => Exec["maverick-systemctl-daemon-reload"],
    }
    file { "/srv/maverick/software/maverick/bin/apsitl.sh":
        ensure      => link,
        target      => "/srv/maverick/software/maverick/manifests/maverick-modules/maverick_dev/files/apsitl.sh",
    }

    # Install dronekit
    class { "maverick_dev::dronekit": }

    # Install dronecode
    class { "maverick_dev::dronecore": }

    # Install/compile ardupilot and SITL
    if $ardupilot {
        class { "maverick_dev::ardupilot": 
            sitl    => $apsitl_dev,
        }
    }

    # Create default dev apsitl instance
    if $apsitl_dev {
        class { "maverick_dev::apsitl_dev": }
    }

    # Install/compile px4 and SITL    
    if $px4 == true {
        class { "maverick_dev::px4":
            sitl    => $px4sitl_dev,
        }
    }

}
