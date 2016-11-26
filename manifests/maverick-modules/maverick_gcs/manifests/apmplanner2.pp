class maverick_gcs::apmplanner2 (
) {
    # WIP: This doesn't work yet, at least on odroid ubuntu
    
    oncevcsrepo { "git-apmplanner2":
        gitsource   => "https://github.com/ArduPilot/apm_planner.git",
        dest        => "/srv/maverick/var/build/apmplanner2",
        owner       => "mav",
    }

    ensure_packages(["qt5-qmake", "qt5-default", "qtscript5-dev", "libqt5webkit5-dev", "libqt5serialport5-dev", "libqt5svg5-dev", "qtdeclarative5-qtquick2-plugin"])
    ensure_packages(["libsdl1.2-dev", "libsndfile1-dev", "flite1-dev", "libssl-dev", "libudev-dev", "libsdl2-dev"])

    exec { "apmplanner-qmake":
        command     => "/usr/bin/qmake apm_planner.pro",
        cwd         => "/srv/maverick/var/build/apmplanner2",
        creates     => "/srv/maverick/var/build/apmplanner2/Makefile",
        user        => "mav",
    } ->
    exec { "apmplanner-make":
        command     => "/usr/bin/make",
        cwd         => "/srv/maverick/var/build/apmplanner2",
        creates     => "/srv/maverick/var/build/apmplanner2/release/apmplanner2",
        require     => Package["qt5-qmake", "qt5-default", "qtscript5-dev", "libqt5webkit5-dev", "libqt5serialport5-dev", "libqt5svg5-dev", "qtdeclarative5-qtquick2-plugin", "libsdl1.2-dev", "libsndfile1-dev", "flite1-dev", "libssl-dev", "libudev-dev", "libsdl2-dev"],
        user        => "mav",
    }
    
}