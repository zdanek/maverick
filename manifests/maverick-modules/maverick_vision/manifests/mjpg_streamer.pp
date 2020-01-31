# @summary
#   Maverick_vision::mjpg_streamer class
#   This class installs and manages the mjpg_streamer library.
#
# @example Declaring the class
#   This class is included from maverick_vision class and should not be included from elsewhere
#
class maverick_vision::mjpg_streamer {
    
    ensure_packages(["cmake", "libjpeg-dev"])
    
    # Add mjpg-streamer repo from git
    oncevcsrepo { "git-mjpg-streamer":
        gitsource   => "https://github.com/jacksonliam/mjpg-streamer.git",
        dest        => "/srv/maverick/software/mjpg-streamer",
    } ->
    exec { "mjpg-streamer-compile":
        user        => "mav",
        timeout     => 0,
        require     => Package["cmake", "libjpeg-dev"],
        command     => "/usr/bin/make -j${::processorcount} >/srv/maverick/var/log/build/mjpg-streamer.build.log 2>&1",
        cwd         => "/srv/maverick/software/mjpg-streamer/mjpg-streamer-experimental",
        creates     => "/srv/maverick/software/mjpg-streamer/mjpg-streamer-experimental/mjpg_streamer",
    }
    
}
