# @summary
#   Maverick_dev class
#   This class manages the development environment.
#
# @example Declaring the class
#   This class is included from the environment manifests and is not usually included elsewhere.
#   It could be included selectively from eg. minimal environment.
#   Note it needs to be included in the flight environment so it can *control* the dev environment - ie. turn off dev services/components.
#
# @param ardupilot
#   If set to true, include the maverick_dev::ardupilot class which clones and compiles the Ardupilot firmware, both for local development and uploading to flight controllers.
# @param apsitl_apdev
#   If set to true, set up the default Ardupilot SITL instance.
# @param px4
#   If set to true, include the maverick_dev::px4 class which clones and compiles the PX4Pro firmware.
# @param px4sitl_dev
#   If set to true, setup the PX4 SITL environment
# @param file_watchers
#   The max number of OS inotify file watchers that are allowed.  This allows IDEs to track more files, at the expense of kernel/system memory.
#
class maverick_dev (
    Boolean $apsitl_apdev = true,
    Boolean $ardupilot = true,
    Boolean $px4 = true,
    Boolean $px4sitl_px4dev = true,
    Integer $file_watchers = 8192,
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

    # Install/compile ardupilot and SITL
    if $ardupilot == true {
        class { "maverick_dev::ardupilot": 
            sitl    => $apsitl_apdev,
        }
    }

    # Create default dev apsitl instance
    if $apsitl_apdev == true {
        class { "maverick_dev::apsitl_apdev": }
    }

    # Install/compile px4 and SITL    
    if $px4 == true {
        class { "maverick_dev::px4":
            sitl    => $px4sitl_px4dev,
        }
    }

    # Increase kernel inotify watcher limits
    base::sysctl::conf { 
        "fs.inotify.max_user_watches": 	value => $file_watchers;
    }

}
