class maverick_gcs::qgroundcontrol (
) {
    # WIP: This doesn't work yet, at least on odroid ubuntu

    oncevcsrepo { "git-qgc":
        gitsource   => "https://github.com/mavlink/qgroundcontrol.git",
        dest        => "/srv/maverick/var/build/qgroundcontrol",
        owner       => "mav",
        submodules  => true,
    }
    ensure_packages(["espeak", "libespeak-dev", "libudev-dev", "libsdl2-dev", "qt5-default"])
    
}