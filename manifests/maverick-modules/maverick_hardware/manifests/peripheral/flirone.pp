class maverick_hardware::peripheral::flirone (
    $active = false,
) {
    
    # Install dependencies
    if $::raspberry_present == "yes" {
        ensure_packages(["raspberrypi-kernel-headers"])
    }
    ensure_packages(["libusb-1.0-0-dev", "dkms", "v4l2loopback-utils", "v4l2loopback-dkms", "libjpeg-dev"])
    /*
    if $::operatingsystem == "Debian" {
        exec { "flirone-libturbojpeg-install":
            command     => "/usr/bin/apt install -y libturbojpeg-dev",
            #unless      => "/usr/bin/dpkg -l libturbojpeg-dev",
            creates     => "/usr/include/turbojpeg.h",
            before      => Exec["flirone-compile"],
        }
    } elsif $::operatingsystem == "Ubuntu" {
        ensure_packages(["libjpeg-turbo8-dev"], {'before' => Exec["flirone-compile"]})
    }
    */
    oncevcsrepo { "git-libseek":
        gitsource   => "https://github.com/fnoop/flirone-v4l2.git",
        dest        => "/srv/maverick/software/flirone",
    } ->
    exec { "flirone-compile":
        command     => "/usr/bin/make",
        cwd         => "/srv/maverick/software/flirone",
        creates     => "/srv/maverick/software/flirone/flirone",
        require     => [ Package["v4l2loopback-dkms"], Package["libusb-1.0-0-dev"], Package["libjpeg-dev"] ],
    } ->
    file { "/srv/maverick/software/maverick/bin/flirone_v4l2.sh":
        ensure      => symlink,
        target      => "/srv/maverick/software/maverick/manifests/maverick-modules/maverick_hardware/files/flirone_v4l2.sh",
    } ->
    file { "/etc/systemd/system/maverick-flirone.service":
        ensure      => present,
        source      => "puppet:///modules/maverick_hardware/maverick-flirone.service",
        notify      => Exec["maverick-systemctl-daemon-reload"],
    }
    
    if $active == true {
        service { "maverick-flirone":
            ensure      => running,
            enable      => true,
            require     => Exec["maverick-systemctl-daemon-reload"],
        }
    } else {
        service { "maverick-flirone":
            ensure      => stopped,
            enable      => false,
            require     => Exec["maverick-systemctl-daemon-reload"],
        }
    }

}