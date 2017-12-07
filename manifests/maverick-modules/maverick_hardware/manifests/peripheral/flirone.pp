class maverick_hardware::peripheral::flirone (
) {
    
    ensure_packages(["raspberrypi-kernel-headers", "libusb-1.0-0-dev", "dkms", "v4l2loopback-utils", "v4l2loopback-dkms"])
    oncevcsrepo { "git-libseek":
        gitsource   => "https://github.com/fnoop/flirone-v4l2.git",
        dest        => "/srv/maverick/software/flirone",
    } ->
    exec { "flirone-compile":
        command     => "/usr/bin/make",
        cwd         => "/srv/maverick/software/flirone",
        creates     => "/srv/maverick/software/flirone/flirone",
        require     => [ Package["v4l2loopback-dkms"], Package["libusb-1.0-0-dev"] ],
    }
    
}