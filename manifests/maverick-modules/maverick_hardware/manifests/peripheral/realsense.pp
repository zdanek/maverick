class maverick_hardware::peripheral::realsense (
    $sdk1 = false,
    $sdk2 = true,
) {

    require base::python

    if $sdk1 == true {
        ensure_packages(["libglfw3", "libglfw3-dev", "pkg-config", "libssl-dev", "liblz4-dev", "liblog4cxx-dev", "libgtk-3-dev", "libglu1-mesa-dev", "freeglut3-dev"])

        if ! ("install_flag_realsense-legacy" in $installflags) {
            # Clone source from github
            oncevcsrepo { "git-realsense-librealsense":
                gitsource   => "https://github.com/IntelRealSense/librealsense.git",
                dest        => "/srv/maverick/var/build/realsense-legacy",
                revision    => "legacy",
            } ->
            # Create build directory
            file { "/srv/maverick/var/build/realsense-legacy/build":
                ensure      => directory,
                owner       => "mav",
                group       => "mav",
                mode        => "755",
            } ->
            # Build and install
            exec { "realsense-legacy-prepbuild":
                user        => "mav",
                timeout     => 0,
                command     => "/usr/bin/cmake -DBUILD_EXAMPLES=true -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/realsense-legacy -DCMAKE_INSTALL_RPATH=/srv/maverick/software/realsense-legacy/lib ..",
                cwd         => "/srv/maverick/var/build/realsense-legacy/build",
                creates     => "/srv/maverick/var/build/realsense-legacy/build/Makefile",
                require     => [ File["/srv/maverick/var/build/realsense-legacy/build"], Package["libglfw3-dev"], Package["libusb-1.0-0-dev"] ], # ensure we have all the dependencies satisfied
            } ->
            exec { "realsense-legacy-build":
                user        => "mav",
                timeout     => 0,
                command     => "/usr/bin/make -j${buildparallel} >/srv/maverick/var/log/build/realsense-legacy.build.out 2>&1",
                cwd         => "/srv/maverick/var/build/realsense-legacy/build",
                creates     => "/srv/maverick/var/build/realsense-legacy/build/librealsense.so",
                require     => Exec["realsense-legacy-prepbuild"],
            } ->
            exec { "realsense-legacy-install":
                user        => "mav",
                timeout     => 0,
                command     => "/usr/bin/make install >/srv/maverick/var/log/build/realsense-legacy.install.out 2>&1",
                cwd         => "/srv/maverick/var/build/realsense-legacy/build",
                creates     => "/srv/maverick/software/realsense-legacy/lib/librealsense.so",
            } ->
            # Install and activate udev rules
            exec { "realsense-legacy-cp-udevbin":
                command     => "/bin/cp /srv/maverick/var/build/realsense-legacy/config/usb-R200* /usr/local/bin && chmod a+rx /usr/local/bin/usb-R200*",
                creates     => "/usr/local/bin/usb-R200-in",
            } ->
            exec { "realsense-legacy-cp-udev":
                command     => "/bin/cp /srv/maverick/var/build/realsense-legacy/config/99-realsense-libusb.rules /etc/udev/rules.d",
                unless      => "/bin/grep '/usr/local/bin/usb' /etc/udev/rules.d/99-realsense-libusb.rules",
                notify      => Exec["realsense-legacy-udev-control"],
            } ->
            exec { "realsense-legacy-udev-control":
                command         => "/sbin/udevadm control --reload-rules && /sbin/udevadm trigger",
                refreshonly     => true
            } ->
            file { "/srv/maverick/var/build/.install_flag_realsense-legacy":
                ensure          => file,
                owner           => "mav",
            }
        }
    }

    if $sdk2 == true {
        ensure_packages(["libglfw3", "libglfw3-dev", "libusb-1.0-0-dev", "pkg-config", "libssl-dev", "libgtk-3-dev", "libgl1-mesa-dev", "libglu1-mesa-dev"])
        ensure_packages(["libprotobuf-dev", "libtbb-dev"])

        # Install cmake path
        file { "/etc/profile.d/70-maverick-realsense-sdk2-cmake.sh":
            mode        => "644",
            owner       => "root",
            group       => "root",
            content     => 'NEWPATH="/srv/maverick/software/realsense-sdk2"; if [ -n "${CMAKE_PREFIX_PATH##*${NEWPATH}}" -a -n "${CMAKE_PREFIX_PATH##*${NEWPATH}:*}" ]; then export CMAKE_PREFIX_PATH=$NEWPATH:$CMAKE_PREFIX_PATH; fi',
        }
        file { "/etc/profile.d/70-maverick-realsense-sdk2-path.sh":
            mode        => "644",
            owner       => "root",
            group       => "root",
            content     => 'NEWPATH="/srv/maverick/software/realsense-sdk2/bin"; if [ -n "${PATH##*${NEWPATH}}" -a -n "${PATH##*${NEWPATH}:*}" ]; then export PATH=$NEWPATH:$PATH; fi',
        }
        file { "/etc/profile.d/70-maverick-realsense-sdk2-pkgconfig.sh":
            mode        => "644",
            owner       => "root",
            group       => "root",
            content     => 'NEWPATH="/srv/maverick/software/realsense-sdk2/lib/pkgconfig"; if [ -n "${PKG_CONFIG_PATH##*${NEWPATH}}" -a -n "${PKG_CONFIG_PATH##*${NEWPATH}:*}" ]; then export PKG_CONFIG_PATH=$NEWPATH:$PKG_CONFIG_PATH; fi',
        }
        file { "/etc/profile.d/70-maverick-realsense-sdk2-ldlibrarypath.sh":
            mode        => "644",
            owner       => "root",
            group       => "root",
            content     => 'NEWPATH="/srv/maverick/software/realsense-sdk2/lib"; if [ -n "${LD_LIBRARY_PATH##*${NEWPATH}}" -a -n "${LD_LIBRARY_PATH##*${NEWPATH}:*}" ]; then export LD_LIBRARY_PATH=$NEWPATH:$LD_LIBRARY_PATH; fi',
        }
        file { "/etc/profile.d/70-maverick-realsense-sdk2-pythonpath.sh":
            mode        => "644",
            owner       => "root",
            group       => "root",
            content     => 'NEWPATH="/srv/maverick/software/realsense-sdk2/lib"; if [ -n "${PYTHONPATH##*${NEWPATH}}" -a -n "${PYTHONPATH##*${NEWPATH}:*}" ]; then export PYTHONPATH=$NEWPATH:$PYTHONPATH; fi',
        }
        file { "/etc/ld.so.conf.d/maverick-realsense-sdk2.conf":
            mode        => "644",
            owner       => "root",
            group       => "root",
            content     => "/srv/maverick/software/realsense-sdk2/lib",
            notify      => Exec["maverick-ldconfig"],
        }

        if $::hardwaremodel == "x86_64" and $::operatingsystem == "Ubuntu" {
            if $::lsbdistcodename == "xenial" or $::lsbdistcodename == "bionic" {
                apt::source { 'realsense':
                    location => 'http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo',
                    release  => $::lsbdistcodename,
                    repos    => 'main',
                    key      => {
                        'id'     => 'F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE',
                        'server' => 'keys.gnupg.net',
                    },
                } ->
                package { ["librealsense2-dkms", "librealsense2-utils", "librealsense2-dev", "librealsense2-dbg"]:
                    ensure  => installed,
                }
            }
        }

        if ! ("install_flag_realsense-sdk2" in $installflags) {
            # Clone realsense-sdk
            oncevcsrepo { "git-realsense-realsense_sdk2":
                gitsource   => "https://github.com/IntelRealSense/librealsense.git",
                dest        => "/srv/maverick/var/build/realsense-sdk2",
                revision    => "master",
            }

            if $raspberry_present == "yes" {
                $uvcparam = "-DFORCE_LIBUVC=true"
                # https://github.com/IntelRealSense/librealsense/issues/4565
                exec { "patch-realsense-raspbian":
                    command => "/bin/echo 'set(CMAKE_CXX_FLAGS \"\${CMAKE_CXX_FLAGS} -latomic\")' >>/srv/maverick/var/build/realsense-sdk2/CMakeLists.txt",
                    unless  => "/bin/grep 'CMAKE_CXX_FLAGS' /srv/maverick/var/build/realsense-sdk2/CMakeLists.txt",
                    before  => Exec["realsense-sdk2-prepbuild"],
                    require => Oncevcsrepo["git-realsense-realsense_sdk2"],
                }
                $buildparallel = 1
            } else {
                $uvcparam = ""
                $buildparallel = ceiling((1 + $::processorcount) / 2) # Restrict build parallelization to roughly processors/2 (to restrict memory usage during compilation)
            }

            # Create build directory
            file { "/srv/maverick/var/build/realsense-sdk2/build":
                ensure      => directory,
                owner       => "mav",
                group       => "mav",
                mode        => "755",
                require     => Oncevcsrepo["git-realsense-realsense_sdk2"]
            } ->
            exec { "realsense-sdk2-prepbuild":
                user        => "mav",
                timeout     => 0,
                environment => ["LD_LIBRARY_PATH=/srv/maverick/software/opencv/lib", "PATH=/srv/maverick/software/opencv/bin:/usr/bin:/usr/sbin:/bin:/sbin:/usr/local/sbin", "CMAKE_PREFIX_PATH=/srv/maverick/software/opencv"],
                command     => "/usr/bin/cmake ${uvcparam} -DPYTHON_EXECUTABLE=/srv/maverick/software/python/bin/python3 -DBUILD_PYTHON_BINDINGS=bool:true -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/realsense-sdk2 -DCMAKE_INSTALL_RPATH=/srv/maverick/software/realsense-sdk2/lib:/srv/maverick/software/librealsense/lib .. >/srv/maverick/var/log/build/realsense-sdk2.cmake.out 2>&1",
                cwd         => "/srv/maverick/var/build/realsense-sdk2/build",
                creates     => "/srv/maverick/var/build/realsense-sdk2/build/Makefile",
                require     => [ File["/srv/maverick/var/build/realsense-sdk2/build"], Class["base::python"] ], # ensure we have all the dependencies satisfied
            } ->
            exec { "realsense-sdk2-build":
                user        => "mav",
                timeout     => 0,
                environment => ["CPLUS_INCLUDE_PATH=/srv/maverick/software/realsense-sdk2/include:/srv/maverick/software/opencv/include", "LIBRARY_PATH=/srv/maverick/software/realsense-sdk2/lib:/srv/maverick/software/opencv/lib"],
                command     => "/usr/bin/make -j${buildparallel} >/srv/maverick/var/log/build/realsense-sdk2.build.out 2>&1",
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
            file { "/srv/maverick/var/build/.install_flag_realsense-sdk2":
                ensure      => file,
                owner       => "mav",
            }
        }
    }

}
