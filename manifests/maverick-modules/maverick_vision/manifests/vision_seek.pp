class maverick_vision::vision_seek (
    $active = false,
    $libseek_thermal = true,
    $libseek_thermal_source = "https://github.com/fnoop/libseek-thermal.git",
    $libseek_thermal_revision = "master",
) {
    
    # Install libseek-thermal
    if $libseek_thermal == true {
        ensure_packages(["libusb-1.0-0-dev", "libboost-program-options-dev"])
        if ! ("install_flag_libseek-thermal" in $installflags) {
            oncevcsrepo { "git-libseek-thermal":
                gitsource   => $libseek_thermal_source,
                dest        => "/srv/maverick/var/build/libseek-thermal",
                revision    => $libseek_thermal_revision,
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
                before      => Service_wrapper["maverick-vision_seek"],
            } ->
            file { "/srv/maverick/var/build/.install_flag_libseek-thermal":
                ensure      => present,
                owner       => mav,
                mode        => "644"
            }
        }
        file { "/etc/profile.d/70-maverick-libseek-thermal-pkgconfig.sh":
            mode        => "644",
            owner       => "root",
            group       => "root",
            content     => 'NEWPATH="/srv/maverick/software/libseek-thermal/lib/pkgconfig"; if [ -n "${PKG_CONFIG_PATH##*${NEWPATH}}" -a -n "${PKG_CONFIG_PATH##*${NEWPATH}:*}" ]; then export PKG_CONFIG_PATH=$NEWPATH:$PKG_CONFIG_PATH; fi',
        }
    }

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
    file { "/srv/maverick/config/vision/vision_seek.conf":
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
