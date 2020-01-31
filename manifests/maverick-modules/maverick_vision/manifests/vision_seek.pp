# @summary
#   Maverick_vision::Vision_seek class
#   This class installs and manages software to support the Seek Thermal imagers.
#
# @example Declaring the class
#   This class is included from maverick_vision class and should not be included from elsewhere
#
# @see https://github.com/fnoop/libseek-thermal.git
# 
# @param active
#   If true, start the vision_seek service and enable at boot time.
# @param libseek_thermal
#   If true, install the libseek library that supports the seek thermal hardware.  Should always be true.
# @param libseek_thermal_source
#   Git repo to use to clone/compile/install libseek software.
# @param libseek_thermal_revision
#   Git branch/revision to use to clone/compile/install libseek software.
# 
class maverick_vision::vision_seek (
    Boolean $active = false,
    Boolean $libseek_thermal = true,
    String $libseek_thermal_source = "https://github.com/fnoop/libseek-thermal.git",
    String $libseek_thermal_revision = "master",
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
            # Create build directory
            file { "/srv/maverick/var/build/libseek-thermal/build":
                ensure      => directory,
                owner       => "mav",
                group       => "mav",
                mode        => "755",
            } ->        
            exec { "libseek-thermal-prepbuild":
                user        => "mav",
                timeout     => 0,
                environment => ["LD_LIBRARY_PATH=/srv/maverick/software/opencv/lib", "PATH=/srv/maverick/software/opencv/bin:/usr/bin:/usr/sbin:/bin:/sbin:/usr/local/sbin", "CMAKE_PREFIX_PATH=/srv/maverick/software/opencv", "CMAKE_INSTALL_RPATH=/srv/maverick/software/aruco/lib:/srv/maverick/software/opencv/lib", "PKG_CONFIG_PATH=/srv/maverick/software/opencv/lib/pkgconfig"],
                command     => "/usr/bin/cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/libseek-thermal -DCMAKE_INSTALL_RPATH=/srv/maverick/software/opencv/lib .. >/srv/maverick/var/log/build/libseek-thermal.cmake.out 2>&1",
                cwd         => "/srv/maverick/var/build/libseek-thermal/build",
                creates     => "/srv/maverick/var/build/libseek-thermal/build/Makefile",
                require     => [ Class["maverick_vision::gstreamer"], Class["maverick_vision::opencv"], File["/srv/maverick/var/build/.install_flag_opencv"] ], # ensure we have all the dependencies satisfied
            } ->
            exec { "libseek-thermal-build":
                user        => "mav",
                timeout     => 0,
                command     => "/usr/bin/make -j${::processorcount} >/srv/maverick/var/log/build/libseek-thermal.build.out 2>&1",
                cwd         => "/srv/maverick/var/build/libseek-thermal/build",
                creates     => "/srv/maverick/var/build/libseek-thermal/build/asdf",
                require     => Exec["libseek-thermal-prepbuild"],
            } ->
            exec { "libseek-thermal-install":
                user        => "mav",
                timeout     => 0,
                command     => "/usr/bin/make install >/srv/maverick/var/log/build/libseek-thermal.install.out 2>&1",
                cwd         => "/srv/maverick/var/build/libseek-thermal/build",
                creates     => "/srv/maverick/software/libseek-thermal/bin/asdf",
            } ->
                    /*
            exec { "libseek-thermal-compile":
                user        => "mav",
                timeout     => 0,
                environment => ["CXXFLAGS=-I/srv/maverick/software/opencv/include/opencv4", "LDFLAGS=-L/srv/maverick/software/opencv/lib -Wl,-rpath=/srv/maverick/software/opencv/lib", "PKG_CONFIG_PATH=/srv/maverick/software/opencv/lib/pkgconfig"],
                command     => "/usr/bin/make -j${::processorcount} >/srv/maverick/var/log/build/libseek-thermal.build.log 2>&1",
                cwd         => "/srv/maverick/var/build/libseek-thermal/build",
                creates     => "/srv/maverick/var/build/libseek-thermal/lib/libseek.so",
                require     => [ Package["libusb-1.0-0-dev"], Class["maverick_vision::opencv"] ],
            } ->
            
            exec { "libseek-thermal-install":
                user        => "mav",
                command     => "/usr/bin/make install PREFIX=/srv/maverick/software/libseek-thermal",
                cwd         => "/srv/maverick/var/build/libseek-thermal",
                creates     => "/srv/maverick/software/libseek-thermal/lib/libseek.so",
                before      => Service["maverick-vision_seek"],
            } ->
            */
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
        service { "maverick-vision_seek":
            ensure  => running,
            enable  => true,
        }
    } else {
        service { "maverick-vision_seek":
            ensure  => stopped,
            enable  => false
        }
    }

    # status.d entry
    file { "/srv/maverick/software/maverick/bin/status.d/123.vision/104.vision_seek.status":
        owner   => "mav",
        content => "vision_seek,Seek Thermal Vision\n",
    }
}
