class maverick_hardware::peripheral::seekthermal (
    $libseek_thermal = true,
    $libseek = false,
) {
    
    # Install udev rule so seek thermal can be used by mav user
    file { "/etc/udev/rules.d/95-seekthermal.rules":
        owner       => "root",
        group       => "root",
        mode        => "644",
        content     => "SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"289d\", ATTRS{idProduct}==\"${$seekthermal_modelid}\", MODE=\"0664\", OWNER=\"mav\", GROUP=\"mav\"\n",
    }

    # Install libseek-thermal
    if $libseek_thermal == true {
        ensure_packages(["libusb-1.0-0-dev", "libboost-program-options-dev"])
        if ! ("install_flag_libseek-thermal" in $installflags) {
            oncevcsrepo { "git-libseek-thermal":
                gitsource   => "https://github.com/maartenvds/libseek-thermal.git",
                dest        => "/srv/maverick/var/build/libseek-thermal",
                revision    => "development",
            } ->
            exec { "libseek-thermal-compile":
                user        => "mav",
                timeout     => 0,
                environment => ["CXXFLAGS=-I/srv/maverick/software/opencv/include", "LDFLAGS=-L/srv/maverick/software/opencv/lib -Wl,-rpath=/srv/maverick/software/opencv/lib", "PKG_CONFIG_PATH=/srv/maverick/software/opencv/lib/pkgconfig"],
                command     => "/usr/bin/make -j${::processorcount} >/srv/maverick/var/log/build/libseek-thermal.build.log 2>&1",
                cwd         => "/srv/maverick/var/build/libseek-thermal",
                creates     => "/srv/maverick/var/build/libseek-thermal/lib/libseek.so",
                require     => [ Package["libusb-1.0-0-dev"], Class["maverick_vision::opencv"] ],
            } ->
            exec { "libseek-thermal-install":
                user        => "mav",
                command     => "/usr/bin/make install PREFIX=/srv/maverick/software/libseek-thermal",
                cwd         => "/srv/maverick/var/build/libseek-thermal",
                creates     => "/srv/maverick/software/libseek-thermal/lib/libseek.so",
            } ->
            file { "/srv/maverick/var/build/.install_flag_libseek-thermal":
                ensure      => present,
                owner       => mav,
                mode        => 644
            } ->
            file { "/etc/profile.d/70-maverick-libseek-thermal-pkgconfig.sh":
                mode        => 644,
                owner       => "root",
                group       => "root",
                content     => "export PKG_CONFIG_PATH=/srv/maverick/software/libseek-thermal/lib/pkgconfig:\$PKG_CONFIG_PATH",
            }
        }
    }
    
    # Install libseek
    if $libseek == true {
        if ! ("install_flag_libseek" in $installflags) {
            oncevcsrepo { "git-libseek":
                gitsource   => "https://github.com/zougloub/libseek.git",
                dest        => "/srv/maverick/var/build/libseek",
            } ->
            file { "/srv/maverick/var/build/libseek/waf":
                ensure      => present,
                source      => "puppet:///modules/maverick_hardware/seek-waf",
                owner       => "mav",
                group       => "mav",
                mode        => "755",
            } ->
            file { "/srv/maverick/software/libseek":
                ensure      => directory,
                owner       => "mav",
                group       => "mav",
                mode        => "755",
            } ->
            exec { "prep-libseek":
                command     => "/srv/maverick/var/build/libseek/waf configure --prefix /srv/maverick/software/libseek",
                cwd         => "/srv/maverick/var/build/libseek",
                creates     => "/srv/maverick/var/build/libseek/build/config.log",
                user        => "mav",
            } ->
            exec { "compile-libseek":
                command     => "/srv/maverick/var/build/libseek/waf",
                cwd         => "/srv/maverick/var/build/libseek",
                creates     => "/srv/maverick/var/build/libseek/build/seek-test",
                user        => "mav",
            } ->
            exec { "install-libseek":
                command     => "/srv/maverick/var/build/libseek/waf install --destdir /",
                cwd         => "/srv/maverick/var/build/libseek",
                creates     => "/srv/maverick/software/libseek/lib/libseek.a",
                user        => "mav",
            } ->
            file { "/srv/maverick/var/build/.install_flag_libseek":
                ensure      => present,
                owner       => "mav",
                mode        => 644,
            }
        }
    }
    
}