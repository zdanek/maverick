class maverick_dev::px4 (
    $px4_source = "https://github.com/PX4/Firmware.git",
    $px4_setupstream = true,
    $px4_upstream = "https://github.com/PX4/Firmware.git",
    $px4_branch = "master",
    $sitl = false,
    $sitl_active = true,
    $cross_compile = true,
    $mavlink_proxy = "mavlink-router",
    $mavlink_active = true,
    $mavlink_startingtcp = 5790,
    $mavlink_tcpports = 3,
    $mavlink_startingudp = 14590,
    $mavlink_udpports = 3,
    $mavlink_udpinports = 3,
    $mavlink_serialout = undef,
    $mavlink_outbaud = 115200,
    $mavlink_outflow = false,
    $ros_instance = true,
    $rosmaster_active = true,
    $rosmaster_port = 11315,
    $mavros_active = true,
    $mavros_startup_delay = 10,
    $mavlink_port = 5790,
    $api_instance = true,
) {

    # Install px4 dev/build dependencies
    ensure_packages(["git", "zip", "qtcreator", "cmake", "build-essential", "genromfs", "ninja-build", "openjdk-8-jdk", "gradle", "protobuf-compiler"])

    # Install Gazebo
    ensure_packages(["gazebo7", "libgazebo7-dev"])
    
    # Install px4 python dependencies
    ensure_packages(["python-empy", "python-toml", "python-numpy"])
    install_python_module { 'pip-px4-pandas':
        pkgname     => 'pandas',
        ensure      => present,
    } ->
    install_python_module { 'pip-px4-jinja2':
        pkgname     => 'jinja2',
        ensure      => present,
    } ->
    install_python_module { 'pip-px4-pyserial':
        pkgname     => 'pyserial',
        ensure      => present,
    } ->
    install_python_module { 'pip-px4-pyulog':
        pkgname     => 'pyulog',
        ensure      => present,
    }

    # Install eProsima Fastrtps
    if ! ("install_flag_fastrtps" in $installflags) {
        oncevcsrepo { "git-px4-fastrtps":
            gitsource   => "https://github.com/eProsima/Fast-RTPS",
            dest        => "/srv/maverick/var/build/fastrtps",
        } ->
        file { "/srv/maverick/var/build/fastrtps/build":
            ensure      => directory,
            owner       => "mav",
            group       => "mav",
            mode        => "755",
        } ->
        exec { "fastrtps-cmake":
            command     => "/usr/bin/cmake -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/fastrtps -DTHIRDPARTY=ON -DCOMPILE_EXAMPLES=ON -DBUILD_JAVA=ON .. >/srv/maverick/var/log/build/fastrtps.cmake.log 2>&1",
            user        => "mav",
            timeout     => 0,
            cwd         => "/srv/maverick/var/build/fastrtps/build",
            creates     => "/srv/maverick/var/build/fastrtps/build/Makefile",
            require     => Package["openjdk-8-jdk"],
        } ->
        exec { "fastrtps-make":
            command     => "/usr/bin/make -j2 >/srv/maverick/var/log/build/fastrtps.make.log 2>&1",
            user        => "mav",
            timeout     => 0,
            cwd         => "/srv/maverick/var/build/fastrtps/build",
            creates     => "/srv/maverick/var/build/fastrtps/build/libfastrtps.so",
        } ->
        exec { "fastrtps-install":
            command     => "/usr/bin/make install >/srv/maverick/var/log/build/fastrtps.install.log 2>&1",
            user        => "mav",
            timeout     => 0,
            cwd         => "/srv/maverick/var/build/fastrtps/build",
            creates     => "/srv/maverick/software/fastrtps/lib/libfastrtps.so",
        } ->
        file { "/etc/profile.d/61-maverick-fastrtps-path.sh":
            mode        => "644",
            owner       => "root",
            group       => "root",
            content     => "export PATH=/srv/maverick/software/fastrtps/bin:\$PATH",
        } ->
        file { "/etc/ld.so.conf.d/maverick-fastrtps.conf":
            mode        => "644",
            owner       => "root",
            group       => "root",
            content     => "/srv/maverick/software/fastrtps/lib",
            notify      => Exec["maverick-ldconfig"],
        } ->
        file { "/etc/profile.d/61-maverick-fastrtps-cmake.sh":
            mode        => "644",
            owner       => "root",
            group       => "root",
            content     => "export CMAKE_PREFIX_PATH=\$CMAKE_PREFIX_PATH:/srv/maverick/software/fastrtps",
        } ->
        file { "/srv/maverick/var/build/.install_flag_fastrtps":
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
            require     => Install_python_module['pip-px4-jinja2'],
        } ->
        exec { "px4-make":
            command     => "/usr/bin/make -j2 posix >/srv/maverick/var/log/build/px4.make.log 2>&1",
            environment => ["LD_LIBRARY_PATH=/srv/maverick/software/fastrtps/lib", "PATH=/srv/maverick/software/fastrtps/bin:/usr/bin:/usr/sbin:/bin:/sbin:/usr/local/sbin", "CMAKE_PREFIX_PATH=/srv/maverick/software/fastrtps", "CMAKE_INSTALL_RPATH=/srv/maverick/software/fastrtps/lib"],
            user        => "mav",
            timeout     => 0,
            cwd         => "/srv/maverick/code/px4",
            creates     => "/srv/maverick/code/px4/build/posix_sitl_default/px4",
        }
    }

    if $sitl == true {
        file { "/srv/maverick/var/log/px4sitl":
            ensure      => directory,
            owner       => "mav",
            group       => "mav",
            mode        => "755",
        } ->
        file { "/srv/maverick/var/log/mavlink/px4sitl":
            ensure      => directory,
            owner       => "mav",
            group       => "mav",
            mode        => "755",
        } ->
        exec { "px4-sitl-make":
            command     => "/usr/bin/make -j2 posix_sitl_default >/srv/maverick/var/log/build/px4.sitl.make.log 2>&1",
            environment => ["HEADLESS=1", "LD_LIBRARY_PATH=/srv/maverick/software/fastrtps/lib", "PATH=/srv/maverick/software/fastrtps/bin:/usr/bin:/usr/sbin:/bin:/sbin:/usr/local/sbin", "CMAKE_PREFIX_PATH=/srv/maverick/software/fastrtps", "CMAKE_INSTALL_RPATH=/srv/maverick/software/fastrtps/lib"],
            user        => "mav",
            timeout     => 0,
            cwd         => "/srv/maverick/code/px4",
            creates     => "/srv/maverick/code/px4/build/posix_sitl_default/px4",
        } ->
        file { "/srv/maverick/software/maverick/bin/px4sitl.sh":
            ensure      => link,
            target      => "/srv/maverick/software/maverick/manifests/maverick-modules/maverick_dev/files/px4sitl.sh",
        } ->
        file { "/srv/maverick/config/dev/px4sitl.conf":
            source      => "puppet:///modules/maverick_dev/px4sitl.conf",
            owner       => "mav",
            group       => "mav",
            mode        => "644",
            replace     => false,
            notify      => Service_wrapper["maverick-px4sitl"],
        } ->
        file { "/srv/maverick/config/dev/px4sitl.screen.conf":
            ensure      => present,
            owner       => "mav",
            group       => "mav",
            content     => template("maverick_dev/px4sitl.screen.conf.erb"),
            notify      => Service_wrapper["maverick-px4sitl"],
        } ->
        file { "/etc/systemd/system/maverick-px4sitl.service":
            content     => template("maverick_dev/maverick-px4sitl.service.erb"),
            owner       => "root",
            group       => "root",
            mode        => "644",
            notify      => [ Exec["maverick-systemctl-daemon-reload"], Service_wrapper["maverick-px4sitl"] ]
        }
        if $sitl_active == true {
            service_wrapper { "maverick-px4sitl":
                ensure      => running,
                enable      => true,
                require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-px4sitl.service"] ],
            }
        } else {
            service_wrapper { "maverick-px4sitl":
                ensure      => stopped,
                enable      => false,
                require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-px4sitl.service"] ],
            }
        }
        
        if $ros_instance == true {
            $notifyResources = [ Service_wrapper["maverick-px4sitl"], Service_wrapper["maverick-rosmaster@px4sitl"] ]
        } else {
            $notifyResources = Service_wrapper["maverick-px4sitl"]
        }
        if $mavlink_proxy == "mavproxy" {
            maverick_mavlink::cmavnode { "px4sitl":
                active      => false,
                inputaddress => "udp:0.0.0.0:14550",
                startingudp => $mavlink_startingudp,
                udpports    => $mavlink_udpports,
                udpinports  => $mavlink_udpinports,
                startingtcp => $mavlink_startingtcp,
                tcpports    => $mavlink_tcpports,
                serialout   => $mavlink_serialout,
                outbaud     => $mavlink_outbaud,
                outflow     => $mavlink_outflow,
                notify      => $notifyResources,
            } ->
            maverick_mavlink::mavlink_router { "px4sitl":
                inputtype   => "udp",
                inputaddress => "0.0.0.0",
                inputport   => "14550",
                startingudp => $mavlink_startingudp,
                udpports    => $mavlink_udpports,
                udpinports  => $mavlink_udpinports,
                startingtcp => $mavlink_startingtcp,
                tcpports    => $mavlink_tcpports,
                serialout   => $mavlink_serialout,
                outbaud     => $mavlink_outbaud,
                outflow     => $mavlink_outflow,
                active      => false,
            } ->
            maverick_mavlink::mavproxy { "px4sitl":
                inputaddress => "udp:0.0.0.0:14550",
                instance    => 1,
                startingudp => $mavlink_startingudp,
                udpports    => $mavlink_udpports,
                udpinports  => $mavlink_udpinports,
                startingtcp => $mavlink_startingtcp,
                tcpports    => $mavlink_tcpports,
                serialout   => $mavlink_serialout,
                outbaud     => $mavlink_outbaud,
                outflow     => $mavlink_outflow,
                active      => $mavlink_active,
                notify      => $notifyResources,
            }
        } elsif $mavlink_proxy == "cmavnode" {
            maverick_mavlink::mavproxy { "px4sitl":
                inputaddress => "udp:0.0.0.0:14550",
                instance    => 1,
                startingudp => $mavlink_startingudp,
                udpports    => $mavlink_udpports,
                udpinports  => $mavlink_udpinports,
                startingtcp => $mavlink_startingtcp,
                tcpports    => $mavlink_tcpports,
                serialout   => $mavlink_serialout,
                outbaud     => $mavlink_outbaud,
                outflow     => $mavlink_outflow,
                active      => false,
            } ->
            maverick_mavlink::mavlink_router { "px4sitl":
                inputtype   => "udp",
                inputaddress => "0.0.0.0",
                inputport   => "14550",
                startingudp => $mavlink_startingudp,
                udpports    => $mavlink_udpports,
                udpinports  => $mavlink_udpinports,
                startingtcp => $mavlink_startingtcp,
                tcpports    => $mavlink_tcpports,
                serialout   => $mavlink_serialout,
                outbaud     => $mavlink_outbaud,
                outflow     => $mavlink_outflow,
                active      => false,
            } ->
            maverick_mavlink::cmavnode { "px4sitl":
                inputaddress => "udp:0.0.0.0:14550", # Note cmavnode doesn't support sitl/tcp yet
                startingudp => $mavlink_startingudp,
                udpports    => $mavlink_udpports,
                udpinports  => $mavlink_udpinports,
                startingtcp => $mavlink_startingtcp,
                tcpports    => $mavlink_tcpports,
                serialout   => $mavlink_serialout,
                outbaud     => $mavlink_outbaud,
                outflow     => $mavlink_outflow,
                active      => $mavlink_active,
                notify      => $notifyResources,
            }
        } elsif $mavlink_proxy == "mavlink-router" {
            maverick_mavlink::cmavnode { "px4sitl":
                active      => false,
                inputaddress => "udp:0.0.0.0:14550", # Note cmavnode doesn't support sitl/tcp yet
                startingudp => $mavlink_startingudp,
                udpports    => $mavlink_udpports,
                udpinports  => $mavlink_udpinports,
                startingtcp => $mavlink_startingtcp,
                tcpports    => $mavlink_tcpports,
                serialout   => $mavlink_serialout,
                outbaud     => $mavlink_outbaud,
                outflow     => $mavlink_outflow,
                notify      => $notifyResources,
            } ->
            maverick_mavlink::mavproxy { "px4sitl":
                inputaddress => "udp:0.0.0.0:14550",
                instance    => 1,
                startingudp => $mavlink_startingudp,
                udpports    => $mavlink_udpports,
                udpinports  => $mavlink_udpinports,
                startingtcp => $mavlink_startingtcp,
                tcpports    => $mavlink_tcpports,
                serialout   => $mavlink_serialout,
                outbaud     => $mavlink_outbaud,
                outflow     => $mavlink_outflow,
                active      => false,
            } ->
            maverick_mavlink::mavlink_router { "px4sitl":
                inputtype   => "udp",
                inputaddress => "0.0.0.0",
                inputport   => "14550",
                startingudp => $mavlink_startingudp,
                udpports    => $mavlink_udpports,
                udpinports  => $mavlink_udpinports,
                startingtcp => $mavlink_startingtcp,
                tcpports    => $mavlink_tcpports,
                serialout   => $mavlink_serialout,
                outbaud     => $mavlink_outbaud,
                outflow     => $mavlink_outflow,
                active      => $mavlink_active,
                notify      => $notifyResources,
            }
        }

        # maverick_dev::sitl::ros_instance allows ros to be completely optional
        if $ros_instance == true {
            # Add a ROS master for SITL
            maverick_ros::rosmaster { "px4sitl":
                active  => $rosmaster_active,
                port    => $rosmaster_port,
            } ->
            maverick_ros::mavros { "px4sitl":
                active              => $mavros_active,
                rosmaster_port      => $rosmaster_port,
                mavlink_port        => $mavlink_port,
                mavros_startup_delay => $mavros_startup_delay,
                mavros_launcher     => "px4.launch"
            }
        }
    
    }

    if $cross_compile == true {
        ensure_packages(["python-serial", "openocd", "flex", "bison", "libncurses5-dev", "autoconf", "texinfo", "libftdi-dev", "libtool", "zlib1g-dev"])
        ensure_packages(["gcc-arm-none-eabi", "gdb-arm-none-eabi", "binutils-arm-none-eabi"])
    }

    if $api_instance == true {
        # Create an API instance
        maverick_web::api { "api-px4sitl":
            instance    => "px4sitl",
            active      => true,
            apiport     => 6802,
            rosport     => $rosmaster_port,
        }
    }

}