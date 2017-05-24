class maverick_vision::vision_seek (
) {
    
    # Temp location/copy of files
    file { "/srv/maverick/software/vision_seek":
        ensure  => directory,
        owner   => "mav",
        group   => "mav",
        mode    => "755",
    } ->
    file { "/srv/maverick/software/vision_seek/args.hxx":
        ensure  => present,
        source  => "puppet:///modules/maverick_vision/args.hxx",
        owner   => "mav",
        group   => "mav",
    } ->
    file { "/srv/maverick/software/vision_seek/vision_seek.cpp":
        ensure  => present,
        source  => "puppet:///modules/maverick_vision/vision_seek.cpp",
        owner   => "mav",
        group   => "mav",
    } ->
    file { "/srv/maverick/software/vision_seek/Makefile":
        ensure  => present,
        source  => "puppet:///modules/maverick_vision/Makefile.vision_seek",
        owner   => "mav",
        group   => "mav",
    } ->
    exec { "compile-vision_seek":
        environment => ["PKG_CONFIG_PATH=/srv/maverick/software/libseek-thermal/lib/pkgconfig:/srv/maverick/software/opencv/lib/pkgconfig"],
        command     => "/usr/bin/make",
        cwd         => "/srv/maverick/software/vision_seek",
        creates     => "/srv/maverick/software/vision_seek/vision_seek",
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
    
}