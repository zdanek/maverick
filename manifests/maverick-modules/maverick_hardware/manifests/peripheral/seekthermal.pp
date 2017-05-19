class maverick_hardware::peripheral::seekthermal (
) {
    
    ensure_packages(["libusb-1.0-0-dev"])
    oncevcsrepo { "git-libseek-thermal":
        gitsource   => "https://github.com/maartenvds/libseek-thermal.git",
        dest        => "/srv/maverick/software/libseek-thermal",
    } ->
    exec { "libseek-thermal-compile":
        user        => "mav",
        timeout     => 0,
        environment => ["CXXFLAGS=-I/srv/maverick/software/opencv/include", "LDFLAGS=-L/srv/maverick/software/opencv/lib -R/srv/maverick/software/opencv/lib", "PKG_CONFIG_PATH=/srv/maverick/software/opencv/lib/pkgconfig"],
        command     => "/usr/bin/make -j${::processorcount} >/srv/maverick/var/log/build/libseek-thermal.build.log 2>&1",
        cwd         => "/srv/maverick/software/libseek-thermal",
        creates     => "/srv/maverick/software/libseek-thermal/lib/libseek.a",
        require     => [ Package["libusb-1.0-0-dev"], Class["maverick_vision::opencv"] ],
    } ->
    file { "/etc/udev/rules.d/95-seekthermal.rules":
        owner       => "root",
        group       => "root",
        mode        => "644",
        content     => "SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"289D\", ATTRS{idProduct}==\"${$seekthermal_modelid}\", MODE=\"0666\", GROUP=\"users\"\n",
    }
}