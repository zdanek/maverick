# @summary
#   Maverick_vision::Apriltag class
#   This class installs and manages the AprilTag software.
#
# @example Declaring the class
#   This class is included from maverick_vision class and should not be included from elsewhere
#
# @param github_repo
#   Git repo to use to compile/install AprilTag
# @param github_branch
#   Which git branch to use to compile/install AprilTag
#

class maverick_vision::apriltag (
  $github_repo = "https://github.com/AprilRobotics/apriltag",
  $github_branch = "3.1.2",
) {
    if ! ("install_flag_apriltag" in $installflags) {
        oncevcsrepo { "apriltag-gitclone":
            gitsource   => $github_repo,
            revision    => $github_branch,
            dest        => "/srv/maverick/var/build/apriltag",
        } ->
        exec { "apriltag-cmake":
            user        => "mav",
            command     => "/usr/bin/cmake -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/apriltag -DCMAKE_INSTALL_RPATH=/srv/maverick/software/apriltag/lib . >/srv/maverick/var/log/build/apriltag.cmake.out 2>&1",
            cwd         => "/srv/maverick/var/build/apriltag",
            creates     => "/srv/maverick/var/build/apriltag/CMakeFiles",
        } ->
        exec { "apriltag-make":
            user        => "mav",
            timeout     => 0,
            command     => "/usr/bin/make >/srv/maverick/var/log/build/apriltag.make.out 2>&1",
            cwd         => "/srv/maverick/var/build/apriltag",
            creates     => "/srv/maverick/var/build/apriltag/apriltag_demo",
        } ->
        exec { "apriltag-install":
            user        => "mav",
            timeout     => 0,
            environment => ["PREFIX=/srv/maverick/software/apriltag"],
            command     => "/usr/bin/make install >/srv/maverick/var/log/build/apriltag.install.out 2>&1",
            cwd         => "/srv/maverick/var/build/apriltag",
            creates     => "/srv/maverick/software/apriltag/bin/apriltag_demo",
        } ->
        file { "/srv/maverick/var/build/.install_flag_apriltag":
            ensure      => present,
            owner       => "mav",
        }
    }

    file { "/etc/profile.d/61-maverick-apriltag-path.sh":
        mode        => "644",
        owner       => "root",
        group       => "root",
        content     => 'NEWPATH="/srv/maverick/software/apriltag/bin"; export PATH=${PATH:-${NEWPATH}}; if [ -n "${PATH##*${NEWPATH}}" -a -n "${PATH##*${NEWPATH}:*}" ]; then export PATH=$NEWPATH:$PATH; fi',
    } ->
    file { "/etc/profile.d/61-maverick-apriltag-pkgconfig.sh":
        mode        => "644",
        owner       => "root",
        group       => "root",
        content     => 'NEWPATH="/srv/maverick/software/apriltag/lib/pkgconfig"; export PKG_CONFIG_PATH=${PKG_CONFIG_PATH:-${NEWPATH}}; if [ -n "${PKG_CONFIG_PATH##*${NEWPATH}}" -a -n "${PKG_CONFIG_PATH##*${NEWPATH}:*}" ]; then export PKG_CONFIG_PATH=$NEWPATH:$PKG_CONFIG_PATH; fi',
    } ->
    file { "/etc/profile.d/61-maverick-apriltag-cmake.sh":
        mode        => "644",
        owner       => "root",
        group       => "root",
        content     => 'NEWPATH="/srv/maverick/software/apriltag"; export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH:-${NEWPATH}}; if [ -n "${CMAKE_PREFIX_PATH##*${NEWPATH}}" -a -n "${CMAKE_PREFIX_PATH##*${NEWPATH}:*}" ]; then export CMAKE_PREFIX_PATH=$NEWPATH:$CMAKE_PREFIX_PATH; fi',
    } ->
    file { "/etc/ld.so.conf.d/maverick-apriltag.conf":
        mode        => "644",
        owner       => "root",
        group       => "root",
        content     => "/srv/maverick/software/apriltag/lib",
        notify      => Exec["maverick-ldconfig"],
    }

}
