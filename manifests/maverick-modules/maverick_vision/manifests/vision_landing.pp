# @summary
#   Maverick_vision::Vision_landing class
#   This class installs and manages the GoodRobots Vision_Landing software.
#
# @example Declaring the class
#   This class is included from maverick_vision class and should not be included from elsewhere
#
# @see https://github.com/goodrobots/vision_landing
# 
# @param active
#   If true, start the vision_landing service and enable at boot time.
# @param vision_landing_source
#   The git repo to use to compile and install the vision_landing software.
# @param vision_landing_revision
#   The git branch/revision to use to compile and install the software.
# 
class maverick_vision::vision_landing (
    Boolean $active = false,
    String $vision_landing_source = "https://github.com/goodrobots/vision_landing.git",
    String $vision_landing_revision = "master",
) {

    # Ensure gstreamer resources are applied before this class
    require maverick_vision::gstreamer
    require maverick_vision::opencv
    require maverick_vision::aruco
    
    # Install vision_landing
    oncevcsrepo { "git-vision_landing":
        gitsource   => $vision_landing_source,
        dest        => "/srv/maverick/software/vision_landing",
        revision    => $vision_landing_revision,
        depth       => 0,
    } ->
    # Compile vision_landing
    exec { "vision_landing-compile":
        user        => mav,
        environment => ["LD_LIBRARY_PATH=/srv/maverick/software/opencv/lib:/srv/maverick/software/aruco/lib", "CMAKE_PREFIX_PATH=/srv/maverick/software/opencv:/srv/maverick/software/aruco:/srv/maverick/software/librealsense", "CMAKE_INSTALL_RPATH=/srv/maverick/software/aruco/lib:/srv/maverick/software/opencv/lib:/srv/maverick/software/librealsense"],
        cwd         => "/srv/maverick/software/vision_landing/src",
        command     => "/usr/bin/cmake -Daruco_DIR=/srv/maverick/software/aruco -DOpenCV_DIR=/srv/maverick/software/opencv -DCMAKE_INSTALL_RPATH=/srv/maverick/software/aruco/lib:/srv/maverick/software/opencv/lib -DCMAKE_MODULE_PATH=/srv/maverick/software/opencv:/srv/maverick/software/aruco . && make && make install",
        creates     => "/srv/maverick/software/vision_landing/track_targets",
        require     => [ Class["maverick_vision::opencv"], Class["maverick_vision::aruco"] ],
    } ->
    # Install systemd manifest
    file { "/etc/systemd/system/maverick-vision_landing.service":
        source      => "puppet:///modules/maverick_vision/vision_landing.service",
        owner       => root,
        group       => root,
    } ->
    # Place a default config file
    file { "/srv/maverick/config/vision/vision_landing.conf":
        owner       => "mav",
        group       => "mav",
        mode        => "644",
        content     => template("maverick_vision/vision_landing.conf.erb"),
        replace     => false,
    }
    
    # Create data and log directories
    file { ["/srv/maverick/data/vision/vision_landing", "/srv/maverick/var/log/vision/vision_landing"]:
        owner       => "mav",
        group       => "mav",
        mode        => "755",
        ensure      => directory,
    }

    # Activate or inactivate service
    if $active == true {
        service { "maverick-vision_landing":
            ensure  => running,
            enable  => true,
            require => Exec["vision_landing-compile"]
        }
    } else {
        service { "maverick-vision_landing":
            ensure  => stopped,
            enable  => false
        }
    }

    # status.d entry
    file { "/srv/maverick/software/maverick/bin/status.d/123.vision/103.vision-landing.status":
        owner   => "mav",
        content => "vision_landing,Precision Landing\n",
    }
}
