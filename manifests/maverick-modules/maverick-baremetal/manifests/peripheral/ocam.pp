class maverick-baremetal::peripheral::ocam (
) {
    
    # Add ocam software from git
    ensure_packages(["qt4-default", "libv4l-dev", "libudev-dev"])
    file { "/srv/maverick/software/odroid-ocam":
        ensure 		=> directory,
        require		=> File["/srv/maverick/software"],
        mode		=> 755,
        owner		=> "mav",
        group		=> "mav",
    } ->
    vcsrepo { "/srv/maverick/software/odroid-ocam":
        ensure		=> present,
        provider 	=> git,
        source		=> "https://github.com/withrobot/oCam.git",
        revision	=> "master",
        owner		=> "mav",
        group		=> "mav",
    } ->
    exec { "ocam-viewer-compile":
        user        => "mav",
        timeout     => 0,
        command     => "/usr/bin/qmake && /usr/bin/make -j${::processorcount} release >/srv/maverick/data/logs/build/ocam-viewer.build.log 2>&1",
        cwd         => "/srv/maverick/software/odroid-ocam/oCam_viewer",
        creates     => "/srv/maverick/software/odroid-ocam/oCam_viewer/oCam-viewer",
    } ->
    file { "/srv/maverick/software/maverick/bin/ocam-viewer":
        ensure      => link,
        target      => "/srv/maverick/software/odroid-ocam/oCam_viewer/oCam-viewer",
    }
    
}