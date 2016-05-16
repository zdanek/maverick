class maverick-vision::fpv::mjpg-streamer {
    
    ensure_packages(["cmake", "libjpeg-dev"])
    
    # Add ocam_viewer repo from git
    file { "/srv/maverick/software/mjpg-streamer":
        ensure 		=> directory,
        require		=> File["/srv/maverick/software"],
        mode		=> 755,
        owner		=> "mav",
        group		=> "mav",
    } ->
    oncevcsrepo { "git-mjpg-streamer":
        gitsource   => "https://github.com/jacksonliam/mjpg-streamer.git",
        dest        => "/srv/maverick/software/mjpg-streamer",
    } ->
    exec { "mjpg-streamer-compile":
        user        => "mav",
        timeout     => 0,
        require     => Package["cmake", "libjpeg-dev"],
        command     => "/usr/bin/make -j${::processorcount} >/srv/maverick/data/logs/build/mjpg-streamer.build.log 2>&1",
        cwd         => "/srv/maverick/software/mjpg-streamer/mjpg-streamer-experimental",
        creates     => "/srv/maverick/software/mjpg-streamer/mjpg-streamer-experimental/mjpg_streamer",
    }
    
}