# @summary
#   Maverick_dev::px4 class
#   This class installs/manages the PX4 software/firmware environment.
#
# @example Declaring the class
#   This class is included from maverick_dev class and should not be included from elsewhere
#   It could be included selectively from eg. minimal environment.
#
# @param px4_source
#   Git repo to use to compile PX4 firmware
# @param px4_setupstream
#   If true, set the upstream repo.  Useful if using a forked repo for upstream updates and PRs.
# @param px4_upstream
#   Upstream Git repo.  Should usually not be changed.
# @param px4_branch
#   Git branch to use when compiling PX4.
# @param rtps_branch
#   Git branch to use when compiling FastRTPS.
# @param sitl
#   If true, setup the PX4 SITL environment
#
class maverick_dev::px4 (
    String $px4_source = "https://github.com/PX4/Firmware.git",
    Boolean $px4_setupstream = true,
    String $px4_upstream = "https://github.com/PX4/Firmware.git",
    String $px4_branch = "v1.10.1",
    String $rtps_branch = "v1.8.2",
    Boolean $sitl = false,
    Boolean $sitl_active = false,
    Boolean $cross_compile = true,
    String $mavlink_proxy = "mavlink-router",
    Boolean $mavlink_logging = false,
    Boolean $mavlink_active = false,
    Integer $mavlink_startingtcp = 5790,
    Integer $mavlink_tcpports = 3,
    Integer $mavlink_startingudp = 14590,
    Integer $mavlink_udpports = 3,
    Integer $mavlink_udpinports = 3,
    Optional[String] $mavlink_serialout = undef,
    Integer $mavlink_outbaud = 115200,
    Boolean $mavlink_outflow = false,
    Boolean $mavlink_replaceconfig = true,
    Boolean $ros_instance = true,
    Boolean $rosmaster_active = false,
    Integer $rosmaster_port = 11315,
    Boolean $mavros_active = false,
    Integer $mavros_startup_delay = 10,
    Integer $mavlink_port = 5790,
    Boolean $api_instance = true,
    Boolean $api_active = false,
    Boolean $api_devmode = false,
    Boolean $api_debug = false,
    Boolean $api_replaceconfig = true,
    String $status_priority = "152",
    Boolean $status_entries = true,
) {

    # Install px4 dev/build dependencies
    ensure_packages(["zip", "qtcreator", "cmake", "build-essential", "genromfs", "ninja-build", "openjdk-8-jdk", "gradle", "protobuf-compiler"])

    # Install Gazebo
    case $::operatingsystem {
        "Ubuntu": {
            case $::lsbdistcodename {
                "bionic": {
                    ensure_packages(["gazebo9", "libgazebo9-dev"])
                }
                default: {
                    ensure_packages(["gazebo7", "libgazebo7-dev"])
                }
            }
        }
        default: {
            ensure_packages(["gazebo7", "libgazebo7-dev"])
        }
    }
    
    # Install px4 python dependencies
    ensure_packages(["python-empy", "python-toml", "python-numpy"])
    install_python_module { 'pip-px4-pandas':
        pkgname     => 'pandas',
        ensure      => present,
        timeout     => 0,
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
    } ->
    install_python_module { 'pip-px4-empy':
        pkgname     => 'empy',
        ensure      => present,
    }

    # Install eProsima Fastrtps
    if ! ("install_flag_fastrtps" in $installflags) {
        oncevcsrepo { "git-px4-fastrtps":
            gitsource   => "https://github.com/eProsima/Fast-RTPS",
            dest        => "/srv/maverick/var/build/fastrtps",
            revision	=> $rtps_branch,
            submodules  => true,
            owner       => "mav",
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
            dest        => "/srv/maverick/software/px4",
            revision	=> $px4_branch,
            submodules  => true,
            owner       => "mav",
        } ->
        # If a custom px4 repo is specified, configure the upstream automagically
        exec { "px4_setupstream":
            command     => "/usr/bin/git remote add upstream ${px4_upstream}",
            unless      => "/usr/bin/git remote -v | /bin/grep ${px4_upstream}",
            cwd         => "/srv/maverick/software/px4",
            require     => Install_python_module['pip-px4-jinja2'],
        } ->
        exec { "px4-make":
            command     => "/usr/bin/make -j2 posix >/srv/maverick/var/log/build/px4.make.log 2>&1",
            environment => ["PYTHONPATH=/opt/ros/melodic/lib/python2.7/dist-packages", "PYTHON_EXECUTABLE=/srv/maverick/software/python/bin/python3", "LD_LIBRARY_PATH=/srv/maverick/software/fastrtps/lib", "PATH=/srv/maverick/software/fastrtps/bin:/srv/maverick/software/python/bin:/usr/bin:/usr/sbin:/bin:/sbin:/usr/local/sbin", "CMAKE_PREFIX_PATH=/srv/maverick/software/fastrtps", "CMAKE_INSTALL_RPATH=/srv/maverick/software/fastrtps/lib"],
            user        => "mav",
            timeout     => 0,
            cwd         => "/srv/maverick/software/px4",
            creates     => "/srv/maverick/software/px4/build/px4_sitl_default/bin/px4",
            require     => [ Install_python_module['pip-px4-pandas'], Install_python_module['pip-px4-jinja2'], Install_python_module['pip-px4-empy'] ],
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
            command     => "/usr/bin/make -j2 px4_sitl_default >/srv/maverick/var/log/build/px4.sitl.make.log 2>&1",
            environment => ["PYTHONPATH=/opt/ros/melodic/lib/python2.7/dist-packages", "PYTHON_EXECUTABLE=/srv/maverick/software/python/bin/python3", "HEADLESS=1", "LD_LIBRARY_PATH=/srv/maverick/software/fastrtps/lib", "PATH=/srv/maverick/software/fastrtps/bin:/usr/bin:/usr/sbin:/bin:/sbin:/usr/local/sbin", "CMAKE_PREFIX_PATH=/srv/maverick/software/fastrtps", "CMAKE_INSTALL_RPATH=/srv/maverick/software/fastrtps/lib"],
            user        => "mav",
            timeout     => 0,
            cwd         => "/srv/maverick/software/px4",
            creates     => "/srv/maverick/software/px4/build/px4_sitl_default/bin/px4",
            require     => [ Install_python_module['pip-px4-pandas'], Install_python_module['pip-px4-jinja2'], Install_python_module['pip-px4-empy'] ],
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
            notify      => Service["maverick-px4sitl"],
        } ->
        file { "/srv/maverick/config/dev/px4sitl.screen.conf":
            ensure      => present,
            owner       => "mav",
            group       => "mav",
            content     => template("maverick_dev/px4sitl.screen.conf.erb"),
            notify      => Service["maverick-px4sitl"],
        } ->
        file { "/etc/systemd/system/maverick-px4sitl.service":
            content     => template("maverick_dev/maverick-px4sitl.service.erb"),
            owner       => "root",
            group       => "root",
            mode        => "644",
            notify      => [ Exec["maverick-systemctl-daemon-reload"], Service["maverick-px4sitl"] ]
        }
        if $sitl_active == true {
            service { "maverick-px4sitl":
                ensure      => running,
                enable      => true,
                require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-px4sitl.service"] ],
            }
        } else {
            service { "maverick-px4sitl":
                ensure      => stopped,
                enable      => false,
                require     => [ Exec["maverick-systemctl-daemon-reload"], File["/etc/systemd/system/maverick-px4sitl.service"] ],
            }
        }
        
        if $ros_instance == true {
            $notifyResources = [ Service["maverick-px4sitl"], Service["maverick-rosmaster@px4sitl"] ]
        } else {
            $notifyResources = Service["maverick-px4sitl"]
        }
        if $mavlink_proxy == "mavproxy" {
            maverick_mavlink::cmavnode { "px4sitl":
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
                replaceconfig => $mavlink_replaceconfig,
            } ->
            maverick_mavlink::mavlink_router { "px4sitl":
                inputtype   => "udp",
                inputaddress => "0.0.0.0",
                inputport   => 14550,
                startingudp => $mavlink_startingudp,
                udpports    => $mavlink_udpports,
                udpinports  => $mavlink_udpinports,
                startingtcp => $mavlink_startingtcp,
                tcpports    => $mavlink_tcpports,
                serialout   => $mavlink_serialout,
                outbaud     => $mavlink_outbaud,
                outflow     => $mavlink_outflow,
                logging     => $mavlink_logging,
                replaceconfig => $mavlink_replaceconfig,
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
                replaceconfig => $mavlink_replaceconfig,
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
                replaceconfig => $mavlink_replaceconfig,
            } ->
            maverick_mavlink::mavlink_router { "px4sitl":
                inputtype   => "udp",
                inputaddress => "0.0.0.0",
                inputport   => 14550,
                startingudp => $mavlink_startingudp,
                udpports    => $mavlink_udpports,
                udpinports  => $mavlink_udpinports,
                startingtcp => $mavlink_startingtcp,
                tcpports    => $mavlink_tcpports,
                serialout   => $mavlink_serialout,
                outbaud     => $mavlink_outbaud,
                outflow     => $mavlink_outflow,
                logging     => $mavlink_logging,
                replaceconfig => $mavlink_replaceconfig,
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
                replaceconfig => $mavlink_replaceconfig,
            }
        } elsif $mavlink_proxy == "mavlink-router" {
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
                notify      => $notifyResources,
                replaceconfig => $mavlink_replaceconfig,
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
                replaceconfig => $mavlink_replaceconfig,
            } ->
            maverick_mavlink::mavlink_router { "px4sitl":
                inputtype   => "udp",
                inputaddress => "0.0.0.0",
                inputport   => 14550,
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
                logging     => $mavlink_logging,
                replaceconfig => $mavlink_replaceconfig,
            }
        }

        # maverick_dev::apsitl_dev::ros_instance allows ros to be completely optional
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
            file { "/srv/maverick/software/maverick/bin/status.d/${status_priority}.px4sitl/102.rosmaster.status":
                owner   => "mav",
                content => "rosmaster@px4sitl,ROS (PX4 SITL)\n",
            }
            file { "/srv/maverick/software/maverick/bin/status.d/${status_priority}.px4sitl/103.mavros.status":
                owner   => "mav",
                content => "mavros@px4sitl,MavROS (PX4 SITL)\n",
            }
        }
    
    }

    if $cross_compile == true {
        ensure_packages(["python-serial", "openocd", "flex", "bison", "libncurses5-dev", "autoconf", "texinfo", "libftdi-dev", "libtool", "zlib1g-dev"])
        ensure_packages(["gcc-arm-none-eabi", "gdb-multiarch", "binutils-arm-none-eabi"])
    }

    if $api_instance == true {
        # Create an API instance
        maverick_web::api { "api-px4sitl":
            instance    => "px4sitl",
            active      => $api_active,
            apiport     => 6802,
            rosport     => $rosmaster_port,
            devmode     => $api_devmode,
            debug       => $api_debug,
            replaceconfig    => $api_replaceconfig,
        }
        file { "/srv/maverick/software/maverick/bin/status.d/${status_priority}.px4sitl/104.api.status":
            owner   => "mav",
            content => "api@px4sitl,MavAPI (PX4 SITL)\n",
        }
    }

    if $status_entries == true {
        # status.d entry for collectd
        file { "/srv/maverick/software/maverick/bin/status.d/${status_priority}.px4sitl/":
            ensure  => directory,
            owner   => "mav",
            mode    => "755",
        }
        file { "/srv/maverick/software/maverick/bin/status.d/${status_priority}.px4sitl/__init__":
            owner   => "mav",
            content => "PX4 SITL\n",
        }
        file { "/srv/maverick/software/maverick/bin/status.d/${status_priority}.px4sitl/100.px4sitl.status":
            owner   => "mav",
            content => "px4sitl,PX4 SITL\n",
        }
        file { "/srv/maverick/software/maverick/bin/status.d/${status_priority}.px4sitl/101.px4sitl.status":
            owner   => "mav",
            content => "mavlink@px4sitl,Mavlink (PX4 SITL)\n",
        }        
    }
}
