# @summary
#   Maverick_hardware::Peripheral::Ocam class
#   This class installs/manages the Odroid Ocam camera support.
#
# @example Declaring the class
#   This class is included from maverick_hardware::odroid class and should not be included from elsewhere
#
class maverick_hardware::peripheral::ocam (
) {
    
    # Add ocam software from git
    ensure_packages(["qt4-default", "libv4l-dev", "libudev-dev"])
    oncevcsrepo { "git-odroid-ocam":
        gitsource   => "git://github.com/withrobot/oCam.git",
        dest        => "/srv/maverick/software/odroid-ocam",
    } ->
    exec { "ocam-viewer-compile":
        user        => "mav",
        timeout     => 0,
        command     => "/usr/bin/qmake && /usr/bin/make -j${::processorcount} release >/srv/maverick/var/log/build/ocam-viewer.build.log 2>&1",
        cwd         => "/srv/maverick/software/odroid-ocam/Software/oCam_viewer_Linux_1705",
        creates     => "/srv/maverick/software/odroid-ocam/Software/oCam_viewer_Linux_1705/oCam-viewer",
    } ->
    file { "/srv/maverick/software/maverick/bin/ocam-viewer":
        ensure      => link,
        target      => "/srv/maverick/software/odroid-ocam/Software/oCam_viewer_Linux_1705/oCam-viewer",
    }
    
}
