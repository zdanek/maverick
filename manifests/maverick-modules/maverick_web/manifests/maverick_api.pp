class maverick_web::maverick_api (
) {

    # install python components
    install_python_module { "mavapi-graphene":
        pkgname     => "graphene",
        ensure      => atleast,
        version     => "2.0",
    } ->
    install_python_module { "mavapi-sqlalchemy":
        pkgname     => "SQLAlchemy",
        ensure      => present,
    } ->
    install_python_module { "mavapi-graphene-sqlalchemy":
        pkgname     => "graphene-sqlalchemy",
        ensure      => present,
    } ->
    install_python_module { "mavapi-tornado":
        pkgname     => "tornado",
        ensure      => present,
    } ->
    install_python_module { "mavapi-rx":
        pkgname     => "rx",
        ensure      => present,
    } ->
    install_python_module { "mavapi-zeroconf":
        pkgname     => "zeroconf",
        ensure      => present,
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
    } ->
    exec { "mavros_maverick-wstool-init":
        command     => "/usr/bin/wstool init",
        user        => "mav",
        cwd         => "/srv/maverick/var/build/catkin_mavros_maverick",
        creates     => "/srv/maverick/var/build/catkin_mavros_maverick/.rosinstall",
    } ->
    oncevcsrepo { "git-mavros_maverick":
        gitsource   => "https://github.com/goodrobots/mavros_maverick.git",
        dest        => "/srv/maverick/var/build/catkin_mavros_maverick/src/mavros_maverick",
    } ->
    exec { "mavros_maverick-catkin":
        user        => "mav",
        environment => ["PYTHONPATH=/opt/ros/current/lib/python2.7/dist-packages", "CMAKE_PREFIX_PATH=/opt/ros/kinetic"],
        cwd         => "/srv/maverick/var/build/catkin_mavros_maverick",
        # command     => "/opt/ros/current/bin/catkin_make_isolated --install --install-space /srv/maverick/software/ros/current -DCMAKE_BUILD_TYPE=Release",
        command     => "/opt/ros/current/bin/catkin_make_isolated --install-space /srv/maverick/software/ros/current -DCMAKE_BUILD_TYPE=Release",
        creates     => "/srv/maverick/software/ros/current/lib/libmavros_maverick.so",
    } ->
    exec { "mavros_maverick-install":
        cwd         => "/srv/maverick/var/build/catkin_mavros_maverick/build_isolated/mavros_maverick",
        command     => "/usr/bin/make install",
        creates     => "/srv/maverick/software/ros/current/lib/libmavros_maverick.so",
    }

}