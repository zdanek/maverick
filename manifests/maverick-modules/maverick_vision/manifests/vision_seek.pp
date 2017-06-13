class maverick_vision::vision_seek (
    $active = false,
) {
    
    # Temp location/copy of files
    file { "/srv/maverick/software/vision_seek":
        ensure  => directory,
        owner   => "mav",
        group   => "mav",
        mode    => "755",
    } ->
    file { "/srv/maverick/software/maverick/bin/vision_seek.sh":
        ensure      => link,
        target      => "/srv/maverick/software/maverick/manifests/maverick-modules/maverick_vision/files/vision_seek.sh",
    } ->
    # Place a default config file
    file { "/srv/maverick/data/config/vision/vision_seek.conf":
        owner       => "mav",
        group       => "mav",
        mode        => "644",
        content     => template("maverick_vision/vision_seek.conf.erb"),
        replace     => false,
    } ->
    # Create default area to save video
    file { "/srv/maverick/data/vision/vision_seek":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    } ->
    file { "/etc/systemd/system/maverick-vision_seek.service":
        ensure  => present,
        source  => "puppet:///modules/maverick_vision/vision_seek.service",
        notify  => Exec["maverick-systemctl-daemon-reload"],
    }
    
    # Activate or inactivate service
    if $active == true {
        service_wrapper { "maverick-vision_seek":
            ensure  => running,
            enable  => true,
        }
    } else {
        service_wrapper { "maverick-vision_seek":
            ensure  => stopped,
            enable  => false
        }
    }
}