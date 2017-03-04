class maverick_baremetal::peripheral::realsense (
) {

    ensure_packages(["libglfw3", "libglfw3-dev", "libusb-1.0-0", "libusb-1.0-0-dev", "pkg-config", "libssl-dev"])

    # Clone source from github
    oncevcsrepo { "git-realsense-librealsense":
        gitsource   => "https://github.com/IntelRealSense/librealsense.git",
        dest        => "/srv/maverick/var/build/librealsense",
    } ->
    
    # Create build directory
    file { "/srv/maverick/var/build/librealsense/build":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => 755,
    } ->
    
    # Build and install
    exec { "librealsense-prepbuild":
        user        => "mav",
        timeout     => 0,
        command     => "/usr/bin/cmake -DBUILD_EXAMPLES=true -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/librealsense -DCMAKE_INSTALL_RPATH=/srv/maverick/software/librealsense/lib ..",
        cwd         => "/srv/maverick/var/build/librealsense/build",
        creates     => "/srv/maverick/var/build/librealsense/build/Makefile",
        require     => [ File["/srv/maverick/var/build/librealsense/build"], Package["libglfw3-dev"], Package["libusb-1.0-0-dev"] ], # ensure we have all the dependencies satisfied
    } ->
    exec { "librealsense-build":
        user        => "mav",
        timeout     => 0,
        command     => "/usr/bin/make -j${::processorcount} >/srv/maverick/var/log/build/librealsense.build.out 2>&1",
        cwd         => "/srv/maverick/var/build/librealsense/build",
        creates     => "/srv/maverick/var/build/librealsense/build/librealsense.so",
        require     => Exec["librealsense-prepbuild"],
    } ->
    exec { "librealsense-install":
        user        => "mav",
        timeout     => 0,
        command     => "/usr/bin/make install >/srv/maverick/var/log/build/librealsense.install.out 2>&1",
        cwd         => "/srv/maverick/var/build/librealsense/build",
        creates     => "/srv/maverick/software/librealsense/lib/librealsense.so",
    } ->
    
    # Install and activate udev rules
    exec { "librealsense-cp-udevbin":
        command     => "/bin/cp /srv/maverick/var/build/librealsense/config/usb-R200* /usr/local/bin",
        creates     => "/usr/local/bin/usb-R200-in",
    } ->
    exec { "librealsense-cp-udev":
        command     => "/bin/cp /srv/maverick/var/build/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d",
        creates     => "/etc/udev/rules.d/99-realsense-libusb.rules",
        notify      => Exec["librealsense-udev-control"],
    } ->
    exec { "librealsense-udev-control":
        command         => "/sbin/udevadm control --reload-rules && /sbin/udevadm trigger",
        refreshonly     => true
    }

}