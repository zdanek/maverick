# @summary
#   Maverick_vision class
#   This class controls all other classes in maverick_vision module.
#
# @example Declaring the class
#   This class is included from the environment manifests and is not usually included elsewhere.
#   It could be included selectively from eg. minimal environment.
#
# @param visiond
#   If true, include the maverick_vision::visiond class which manages the Maverick visiond service.
# @param gstreamer
#   If true, include the maverick_vision::gstreamer class which installs/configures gstreamer software.
# @param opencv
#   If true, include the maverick_vision::opencv class which installs/configures OpenCV software.
# @param visionlibs
#   If true, include the maverick_vision::visionlibs class which installs/configures various supporting vision libraries.
# @param mjpg_streamer
#   If true, include the maverick_vision::mjpg_streamer class which installs/configures mjpg_streamer software.
# @param aruco
#   If true, include the maverick_vision::aruco class which installs/configures aruco software.
# @param apriltag
#   If true, include the maverick_vision::apriltag class which installs/configures apriltag software.
# @param orb_slam2
#   If true, include the maverick_vision::orb_slam2 class which installs/configures the orb/slam2 software.
# @param vision_landing
#   If true, include the maverick_vision::vision_landing class which installs/configures the vision_landing software.
# @param vision_seek
#   If true, include the maverick_vision::vision_seek class which installs/configures software for the seek thermal camera.
# @param camera_manager
#   If true, include the maverick_vision::collision_avoidance class which installs/configures the collission avoidance software.
# @param rtabmap
#   If true, include the maverick_vision::rtabmap class which installs/configures the rtabmap software.
#
class maverick_vision (
    Boolean $visiond = true,
    Boolean $gstreamer = true,
    Boolean $opencv = true,
    Boolean $visionlibs = true,
    Boolean $mjpg_streamer = false,
    Boolean $aruco = true,
    Boolean $apriltag = true,
    Boolean $orb_slam2 = false,
    Boolean $vision_landing = false,
    Boolean $vision_seek = true,
    Boolean $camera_manager = false,
    Boolean $collision_avoidance = false,
    Boolean $rtabmap = false,
) {

    file { ["/srv/maverick/config/vision", "/srv/maverick/data/vision", "/srv/maverick/var/log/vision"]:
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    }

    # Create status.d directory for maverick status`
    file { "/srv/maverick/software/maverick/bin/status.d/123.vision":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    } ->
    file { "/srv/maverick/software/maverick/bin/status.d/123.vision/__init__":
        owner       => "mav",
        content     => "Vision Services",
    }
    
    if $visionlibs == true {
        class { "maverick_vision::visionlibs": }
    }

    if $visiond == true {
        class { "maverick_vision::visiond": }
    }

    if $gstreamer == true {
        class { "maverick_vision::gstreamer": }
    }

    if $mjpg_streamer == true  {
        class { "maverick_vision::mjpg_streamer": }
    }

    if $opencv == true {
        class { "maverick_vision::opencv": }
    }

    if $aruco == true {
        class { "maverick_vision::aruco": }
    }

    if $apriltag == true {
        class { "maverick_vision::apriltag": }
    }

    if $orb_slam2 == true {
        class { "maverick_vision::orb_slam2": }
    }

    if $vision_landing == true {
        class { "maverick_vision::vision_landing": }
    }

    if $camera_manager == true {
        class { "maverick_vision::camera_manager": }
    }

    if $vision_seek == true {
        class { "maverick_vision::vision_seek": }
    }
    
    if $collision_avoidance == true {
        class { "maverick_vision::collision_avoidance": }
    }

    if $rtabmap == true {
        class { "maverick_vision::rtabmap": }
    }
}
