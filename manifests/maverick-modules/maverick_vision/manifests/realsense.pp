class maverick_vision::realsense (
    $sdk1 = false,
    $sdk2 = true,
) {

    if $sdk1 == true {
        ensure_packages(["libglfw3", "libglfw3-dev", "libusb-1.0-0", "libusb-1.0-0-dev", "pkg-config", "libssl-dev", "liblz4-dev", "liblog4cxx-dev"])
    
        # Install cmake path    
        file { "/etc/profile.d/70-maverick-librealsense-cmake.sh":
            mode        => "644",
            owner       => "root",
            group       => "root",
            content     => "export CMAKE_PREFIX_PATH=\$CMAKE_PREFIX_PATH:/srv/maverick/software/librealsense",
        }
    
        if ! ("install_flag_realsense_sdk1" in $installflags) {
            # Clone realsense-sdk
            oncevcsrepo { "git-realsense-realsense_sdk":
                gitsource   => "https://github.com/IntelRealSense/realsense_sdk.git",
                dest        => "/srv/maverick/var/build/realsense-sdk",
            } ->
            # Create build directory
            file { "/srv/maverick/var/build/realsense-sdk/build":
                ensure      => directory,
                owner       => "mav",
                group       => "mav",
                mode        => "755",
            } ->
            exec { "realsense-sdk-prepbuild":
                user        => "mav",
                timeout     => 0,
                environment => ["LD_LIBRARY_PATH=/srv/maverick/software/opencv/lib", "PATH=/srv/maverick/software/opencv/bin:/usr/bin:/usr/sbin:/bin:/sbin:/usr/local/sbin", "CMAKE_PREFIX_PATH=/srv/maverick/software/opencv"],
                command     => "/usr/bin/cmake -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/realsense-sdk -DCMAKE_INSTALL_RPATH=/srv/maverick/software/realsense-sdk/lib:/srv/maverick/software/librealsense/lib ..",
                cwd         => "/srv/maverick/var/build/realsense-sdk/build",
                creates     => "/srv/maverick/var/build/realsense-sdk/build/Makefile",
                require     => [ File["/srv/maverick/var/build/realsense-sdk/build"], Package["liblz4-dev"], Package["liblog4cxx-dev"] ], # ensure we have all the dependencies satisfied
            } ->
            exec { "realsense-sdk-build":
                user        => "mav",
                timeout     => 0,
                environment => ["CPLUS_INCLUDE_PATH=/srv/maverick/software/librealsense/include:/srv/maverick/software/opencv/include", "LIBRARY_PATH=/srv/maverick/software/librealsense/lib:/srv/maverick/software/opencv/lib"],
                command     => "/usr/bin/make -j${::processorcount} >/srv/maverick/var/log/build/realsense-sdk.build.out 2>&1",
                cwd         => "/srv/maverick/var/build/realsense-sdk/build",
                creates     => "/srv/maverick/var/build/realsense-sdk/build/sdk/src/core/pipeline/librealsense_pipeline.so",
                require     => Exec["realsense-sdk-prepbuild"],
            } ->
            exec { "realsense-sdk-install":
                user        => "mav",
                timeout     => 0,
                command     => "/usr/bin/make install >/srv/maverick/var/log/build/realsense-sdk.install.out 2>&1",
                cwd         => "/srv/maverick/var/build/realsense-sdk/build",
                creates     => "/srv/maverick/software/realsense-sdk/bin/realsense_fps_counter_sample",
            } ->
            file { "/srv/maverick/var/build/.install_flag_realsense_sdk":
                ensure      => file,
                owner       => "mav",
            }

            # Disable samples for now
            if 1 == 2 {
                # Clone examples source from github
                file { "/srv/maverick/code/realsense":
                    ensure          => directory,
                    owner           => mav,
                    group           => mav,
                    mode            => "755",
                } ->
                oncevcsrepo { "git-realsense-realsense_samples":
                    gitsource   => "https://github.com/IntelRealSense/realsense_samples.git",
                    dest        => "/srv/maverick/code/realsense/samples",
                } ->
                file { "/srv/maverick/code/realsense/samples/build":
                    ensure      => directory,
                    owner       => "mav",
                    group       => "mav",
                    mode        => "755",
                } ->
                exec { "realsense-samples-prepbuild":
                    user        => "mav",
                    timeout     => 0,
                    environment => ["CMAKE_PREFIX_PATH=/srv/maverick/software/opencv:/srv/maverick/software/librealsense"],
                    command     => "/usr/bin/cmake -DCMAKE_MODULE_PATH=/srv/maverick/software/opencv:/srv/maverick/software/librealsense ..",
                    cwd         => "/srv/maverick/code/realsense/samples/build",
                    creates     => "/srv/maverick/code/realsense/samples/build/Makefile",
                } ->
                exec { "realsense-samples-build":
                    user        => "mav",
                    timeout     => 0,
                    environment => ["CPPFLAGS=-I/srv/maverick/software/librealsense/include -I/srv/maverick/software/opencv/include", "CPLUS_INCLUDE_PATH=/srv/maverick/software/librealsense/include:/srv/maverick/software/opencv/include:/srv/maverick/software/realsense-sdk/include", "LIBRARY_PATH=/srv/maverick/software/librealsense/lib:/srv/maverick/software/opencv/lib:/srv/maverick/software/realsense-sdk/lib"],
                    command     => "/usr/bin/make -j${::processorcount} >/srv/maverick/var/log/build/realsense-samples.build.out 2>&1",
                    cwd         => "/srv/maverick/code/realsense/samples/build",
                    # creates     => "/srv/maverick/var/build/realsense-samples/sdk/src/core/pipeline/librealsense_pipeline.so",
                    require     => [ Exec["realsense-samples-prepbuild"], Exec["realsense-sdk-install"] ]
                }
            }
        }
    }
    
    if $sdk2 == true {
        ensure_packages(["libglfw3", "libglfw3-dev", "libusb-1.0-0-dev", "pkg-config", "libssl-dev", "libgtk-3-dev", "libgl1-mesa-dev", "libglu1-mesa-dev"])

        # Install cmake path
        file { "/etc/profile.d/70-maverick-realsense-sdk2-cmake.sh":
            mode        => "644",
            owner       => "root",
            group       => "root",
            content     => "export CMAKE_PREFIX_PATH=\$CMAKE_PREFIX_PATH:/srv/maverick/software/realsense-sdk2",
        }

        if ! ("install_flag_realsense_sdk2" in $installflags) {
            # Clone realsense-sdk
            oncevcsrepo { "git-realsense-realsense_sdk":
                gitsource   => "https://github.com/IntelRealSense/librealsense.git",
                dest        => "/srv/maverick/var/build/realsense-sdk2",
            } ->
            # Create build directory
            file { "/srv/maverick/var/build/realsense-sdk2/build":
                ensure      => directory,
                owner       => "mav",
                group       => "mav",
                mode        => "755",
            } ->
            exec { "realsense-sdk2-prepbuild":
                user        => "mav",
                timeout     => 0,
                environment => ["LD_LIBRARY_PATH=/srv/maverick/software/opencv/lib", "PATH=/srv/maverick/software/opencv/bin:/usr/bin:/usr/sbin:/bin:/sbin:/usr/local/sbin", "CMAKE_PREFIX_PATH=/srv/maverick/software/opencv"],
                command     => "/usr/bin/cmake -DBUILD_GRAPHICAL_EXAMPLES=false -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/realsense-sdk2 -DCMAKE_INSTALL_RPATH=/srv/maverick/software/realsense-sdk2/lib:/srv/maverick/software/librealsense/lib ..",
                cwd         => "/srv/maverick/var/build/realsense-sdk2/build",
                creates     => "/srv/maverick/var/build/realsense-sdk2/build/Makefile",
                require     => File["/srv/maverick/var/build/realsense-sdk2/build"], # ensure we have all the dependencies satisfied
            } ->
            exec { "realsense-sdk2-build":
                user        => "mav",
                timeout     => 0,
                environment => ["CPLUS_INCLUDE_PATH=/srv/maverick/software/librealsense/include:/srv/maverick/software/opencv/include", "LIBRARY_PATH=/srv/maverick/software/librealsense/lib:/srv/maverick/software/opencv/lib"],
                command     => "/usr/bin/make -j${::processorcount} >/srv/maverick/var/log/build/realsense-sdk2.build.out 2>&1",
                cwd         => "/srv/maverick/var/build/realsense-sdk2/build",
                creates     => "/srv/maverick/var/build/realsense-sdk2/build/tools/convert/rs-convert",
                require     => Exec["realsense-sdk2-prepbuild"],
            } ->
            exec { "realsense-sdk2-install":
                user        => "mav",
                timeout     => 0,
                command     => "/usr/bin/make install >/srv/maverick/var/log/build/realsense-sdk2.install.out 2>&1",
                cwd         => "/srv/maverick/var/build/realsense-sdk2/build",
                creates     => "/srv/maverick/software/realsense-sdk2/lib/librealsense2.so",
            } ->
            exec { "realsense-sdk2-udev-rules":
                command     => "/bin/cp config/99-realsense-libusb.rules /etc/udev/rules.d/",
                creates     => "/etc/udev/rules.d/99-realsense-libusb.rules",
                cwd         => "/srv/maverick/var/build/realsense-sdk2",
                notify      => Exec["realsense-sdk2-udev-update"],
            } ->
            exec { "realsense-sdk2-udev-update":
                command         => "/sbin/udevadm control --reload-rules && /sbin/udevadm trigger",
                refreshonly     => true
            } ->
            file { "/srv/maverick/var/build/.install_flag_realsense_sdk2":
                ensure      => file,
                owner       => "mav",
            }
        }

    }
}