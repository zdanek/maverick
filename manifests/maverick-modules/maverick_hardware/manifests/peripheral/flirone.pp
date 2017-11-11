class maverick_hardware::peripheral::flirone (
) {
    
    ensure_packages(["raspberrypi-kernel-headers", "libusb-1.0-0-dev", "dkms", "v4l2loopback-utils", "v4l2loopback-dkms"])
    oncevcsrepo { "git-libseek":
        gitsource   => "https://github.com/fnoop/flirone-v4l2.git",
        dest        => "/srv/maverick/var/build/flirone-v4l2",
    } ->
    exec { "flirone-compile":
        command     => "/usr/bin/make",
        cwd         => "/srv/maverick/var/build/flirone-v4l2",
        creates     => "/srv/maverick/var/build/flirone-v4l2/flirone",
        require     => [ Package["v4l2loopback-dkms"], Package["libusb-1.0-0-dev"] ],
    }
    
}