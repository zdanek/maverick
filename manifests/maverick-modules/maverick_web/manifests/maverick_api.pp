class maverick_web::maverick_api (
) {

    # install python components
    install_python_module { "mavapi-pymavlink":
        pkgname     => "pymavlink",
        ensure      => atleast,
        version     => "2.2.10",
    } ->
    install_python_module { "mavapi-graphene":
        pkgname     => "graphene",
        ensure      => exactly,
        version     => "2.1.3",
    } ->
    install_python_module { "mavapi-sqlalchemy":
        pkgname     => "SQLAlchemy",
        ensure      => exactly,
        version     => "1.2.11",
    } ->
    install_python_module { "mavapi-graphene-sqlalchemy":
        pkgname     => "graphene-sqlalchemy",
        ensure      => exactly,
        version     => "2.1.0",
    } ->
    install_python_module { "mavapi-tornado":
        pkgname     => "tornado",
        ensure      => exactly,
        version     => "5.1.1",
    } ->
    install_python_module { "mavapi-rx":
        pkgname     => "rx",
        ensure      => latest,
    } ->
    install_python_module { "mavapi-zeroconf":
        pkgname     => "zeroconf",
        ensure      => latest,
    } ->
    oncevcsrepo { "git-maverick-api":
        gitsource   => "https://github.com/goodrobots/maverick-api.git",
        dest        => "/srv/maverick/code/maverick-api",
        revision    => "master",
        depth       => undef,
    } ->
    file { "/etc/systemd/system/maverick-api@.service":
        owner       => "root",
        group       => "root",
        mode        => "644",
        source      => "puppet:///modules/maverick_web/maverick-api@.service",
        notify      => Exec["maverick-systemctl-daemon-reload"],
    } -> 
    # Create a symlink to api launch script
    file { "/srv/maverick/software/maverick/bin/api.sh":
        ensure      => link,
        target      => "/srv/maverick/software/maverick/manifests/maverick-modules/maverick_web/files/api.sh",
    }
    
    file { "/srv/maverick/var/log/web/api":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    }
    
    # Add extra mavros modules
    if ! ("install_flag_mavros_maverick" in $installflags) {
        file { ["/srv/maverick/var/build/catkin_mavros_maverick", "/srv/maverick/var/build/catkin_mavros_maverick/src"]:
            ensure      => directory,
            owner       => "mav",
            group       => "mav",
            mode        => "755",
        } ->
        exec { "mavros_maverick-catkin-init":
            command     => "/usr/bin/catkin init",
            user        => "mav",
            cwd         => "/srv/maverick/var/build/catkin_mavros_maverick",
            creates     => "/srv/maverick/var/build/catkin_mavros_maverick/.catkin_tools",
            timeout     => 0,
            require     => Class["maverick_ros"],
        } ->
        exec { "mavros_maverick-wstool-init":
            command     => "/usr/bin/wstool init",
            user        => "mav",
            cwd         => "/srv/maverick/var/build/catkin_mavros_maverick",
            creates     => "/srv/maverick/var/build/catkin_mavros_maverick/.rosinstall",
            timeout     => 0,
        } ->
        oncevcsrepo { "git-mavros_maverick":
            gitsource   => "https://github.com/goodrobots/mavros_maverick.git",
            dest        => "/srv/maverick/var/build/catkin_mavros_maverick/src/mavros_maverick",
        } ->
        exec { "mavros_maverick-catkin":
           # user        => "mav",
            environment => ["PYTHONPATH=/opt/ros/current/lib/python2.7/dist-packages", "CMAKE_PREFIX_PATH=/opt/ros/current"],
            cwd         => "/srv/maverick/var/build/catkin_mavros_maverick",
            command     => "/opt/ros/current/bin/catkin_make_isolated -j1 --install --install-space /srv/maverick/software/ros/current -DCMAKE_BUILD_TYPE=Release",
            creates     => "/srv/maverick/software/ros/current/lib/libmavros_maverick.so",
            timeout     => 0,
        } ->
        exec { "mavros_maverick-install":
            cwd         => "/srv/maverick/var/build/catkin_mavros_maverick/build_isolated/mavros_maverick",
            command     => "/usr/bin/make install",
            creates     => "/srv/maverick/software/ros/current/lib/libmavros_maverick.so",
            timeout     => 0,
        } ->
        file { "/srv/maverick/var/build/.install_flag_mavros_maverick":
            ensure      => present,
            owner       => "mav",
            group       => "mav",
            mode        => "0644",
        }
    }

}