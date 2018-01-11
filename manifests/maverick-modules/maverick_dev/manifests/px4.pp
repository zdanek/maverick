class maverick_dev::px4 (
    $px4_source = "https://github.com/PX4/Firmware.git",
    $px4_setupstream = true,
    $px4_upstream = "https://github.com/PX4/Firmware.git",
    $px4_branch = "master", # eg. master, Copter-3.3, ArduPlane-release
    $sitl = true,
    $cross_compile = false,
) {

    # Install px4 dev/build dependencies
    ensure_packages(["git", "zip", "qtcreator", "cmake", "build-essential", "genromfs", "ninja-build", "openjdk-8-jdk", "gradle"])

    # Install px4 python dependencies
    ensure_packages(["python-empy", "python-toml", "python-numpy"])
    install_python_module { 'pip-px4-pandas':
        pkgname     => 'pandas',
        ensure      => present,
    }
    install_python_module { 'pip-px4-jinja2':
        pkgname     => 'jinja2',
        ensure      => present,
    }
    install_python_module { 'pip-px4-pyserial':
        pkgname     => 'pyserial',
        ensure      => present,
    }
    install_python_module { 'pip-px4-pyulog':
        pkgname     => 'pyulog',
        ensure      => present,
    }

    # Install eProsima FastRTSP
    if ! ("install_flag_fastrtsp" in $installflags) {
        oncevcsrepo { "git-px4-fastrtsp":
            gitsource   => "https://github.com/eProsima/Fast-RTPS",
            dest        => "/srv/maverick/var/build/fastrtsp",
        } ->
        file { "/srv/maverick/var/build/fastrtsp/build":
            ensure      => directory,
            owner       => "mav",
            group       => "mav",
            mode        => "755",
        } ->
        exec { "fastrtsp-cmake":
            command     => "/usr/bin/cmake -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/fastrtsp -DTHIRDPARTY=ON -DCOMPILE_EXAMPLES=ON -DBUILD_JAVA=ON .. >/srv/maverick/var/log/build/fastrtsp.cmake.log 2>&1",
            user        => "mav",
            timeout     => 0,
            cwd         => "/srv/maverick/var/build/fastrtsp/build",
            creates     => "/srv/maverick/var/build/fastrtsp/build/Makefile",
            require     => Package["openjdk-8-jdk"],
        } ->
        exec { "fastrtsp-make":
            command     => "/usr/bin/make -j2 >/srv/maverick/var/log/build/fastrtsp.make.log 2>&1",
            user        => "mav",
            timeout     => 0,
            cwd         => "/srv/maverick/var/build/fastrtsp/build",
            creates     => "/srv/maverick/var/build/fastrtsp/build/libfastrtps.so",
        } ->
        exec { "fastrtsp-install":
            command     => "/usr/bin/make install >/srv/maverick/var/log/build/fastrtsp.install.log 2>&1",
            user        => "mav",
            timeout     => 0,
            cwd         => "/srv/maverick/var/build/fastrtsp/build",
            creates     => "/srv/maverick/software/fastrtsp/lib/libfastrtps.so",
        } ->
        file { "/etc/profile.d/61-maverick-fastrtsp-path.sh":
            mode        => "644",
            owner       => "root",
            group       => "root",
            content     => "export PATH=/srv/maverick/software/fastrtsp/bin:\$PATH",
        } ->
        file { "/etc/ld.so.conf.d/maverick-fastrtsp.conf":
            mode        => "644",
            owner       => "root",
            group       => "root",
            content     => "/srv/maverick/software/fastrtsp/lib",
            notify      => Exec["maverick-ldconfig"],
        } ->
        file { "/etc/profile.d/61-maverick-fastrtsp-cmake.sh":
            mode        => "644",
            owner       => "root",
            group       => "root",
            content     => "export CMAKE_PREFIX_PATH=\$CMAKE_PREFIX_PATH:/srv/maverick/software/fastrtsp",
        } ->
        file { "/srv/maverick/var/build/.install_flag_fastrtsp":
            ensure      => present,
            owner       => "mav",
        }
    }

    # Install PX4 Firmware
    if ! ("install_flag_px4" in $installflags) {
        oncevcsrepo { "git-px4-px4":
            gitsource   => "https://github.com/PX4/Firmware.git",
            dest        => "/srv/maverick/code/px4",
            revision	=> $px4_branch,
            submodules  => true,
        } ->
        # If a custom px4 repo is specified, configure the upstream automagically
        exec { "px4_setupstream":
            command     => "/usr/bin/git remote add upstream ${px4_upstream}",
            unless      => "/usr/bin/git remote -v | /bin/grep ${px4_upstream}",
            cwd         => "/srv/maverick/code/px4",
            require     => Oncevcsrepo["git-ardupilot"],
        } ->
        exec { "px4-make":
            command     => "/usr/bin/make -j2 posix >/srv/maverick/var/log/build/px4.make.log 2>&1",
            environment => ["LD_LIBRARY_PATH=/srv/maverick/software/fastrtsp/lib", "PATH=/srv/maverick/software/fastrtsp/bin:/usr/bin:/usr/sbin:/bin:/sbin:/usr/local/sbin", "CMAKE_PREFIX_PATH=/srv/maverick/software/fastrtsp", "CMAKE_INSTALL_RPATH=/srv/maverick/software/fastrtsp/lib"],
            user        => "mav",
            timeout     => 0,
            cwd         => "/srv/maverick/code/px4",
            creates     => "/srv/maverick/code/px4/build/posix_sitl_default/px4",
        }
    }
    
    if $cross_compile == true {
        ensure_packages(["python-serial", "openocd", "flex", "bison", "libncurses5-dev", "autoconf", "texinfo", "libftdi-dev", "libtool", "zlib1g-dev"])
        ensure_packages(["gcc-arm-none-eabi", "gdb-arm-none-eabi", "binutils-arm-none-eabi"])
    }

}