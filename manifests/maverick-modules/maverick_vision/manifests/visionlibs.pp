# @summary
#   Maverick_vision::Visionlibs class
#   This class installs and manages support libraries for other vision components.
#
# @example Declaring the class
#   This class is included from maverick_vision class and should not be included from elsewhere
#
# @param tbb
#   If true, install Intel TBB multithreading library.
# @param tbb_version
#   Version of TBB to clone, compile and install.
# @param openblas
#   If true, install OpenBLAS library.
# @param openblas_version
#    Version of OpenBLAS to clone, compile and install.
# 
class maverick_vision::visionlibs (
    Boolean $tbb = true,
    String $tbb_version = "v2020.0",
    Boolean $openblas = true,
    String $openblas_version = "v0.3.7",
) {
    
    if $openblas == true {
        # If ~/var/build/.install_flag_openblas exists, skip pulling source and compiling
        if ! ("install_flag_openblas" in $installflags) {
            oncevcsrepo { "git-openblas":
                gitsource   => "https://github.com/xianyi/OpenBLAS.git",
                dest        => "/srv/maverick/var/build/openblas",
                revision    => $openblas_version,
            } ->
            exec { "openblas-make":
                command => "/usr/bin/make >/srv/maverick/var/log/build/openblas.make 2>&1",
                cwd     => "/srv/maverick/var/build/openblas",
                creates => "/srv/maverick/var/build/openblas/libopenblas.so",
                timeout => 0,
                user    => "mav",
            } ->
            exec { "openblas-makeinstall":
                command => "/usr/bin/make install PREFIX=/srv/maverick/software/openblas >/srv/maverick/var/log/build/openblas.install 2>&1",
                cwd     => "/srv/maverick/var/build/openblas",
                creates => "/srv/maverick/software/openblas/lib/libopenblas.so",
                timeout => 0,
                user    => "mav",
            } ->
            file { "/srv/maverick/var/build/.install_flag_openblas":
                ensure  => present,
            }
        }
    }

    if $tbb == true {
        # If ~/var/build/.install_flag_tbb exists, skip pulling source and compiling
        if ! ("install_flag_tbb" in $installflags) {
            oncevcsrepo { "git-tbb":
                gitsource   => "https://github.com/intel/tbb",
                dest        => "/srv/maverick/var/build/tbb",
                revision    => $tbb_version,
            } ->
            exec { "tbb-make":
                command => "/usr/bin/make",
                cwd     => "/srv/maverick/var/build/tbb",
                timeout => 0,
                user    => "mav",
            } ->
            file { "/srv/maverick/software/tbb":
                ensure  => directory,
                owner   => "mav",
                group   => "mav",
            } ->
            file { "/srv/maverick/var/build/tbb/install.sh":
                source  => "puppet:///modules/maverick_vision/tbb_install.sh",
                mode    => "0755",
                owner   => "mav",
                group   => "mav",
            } ->
            exec { "tbb-install":
                command => "/srv/maverick/var/build/tbb/install.sh",
                user    => "mav",
                cwd     => "/srv/maverick/var/build/tbb",
                creates => "/srv/maverick/software/tbb/lib",
            } ->
            file { "/srv/maverick/var/build/.install_flag_tbb":
                ensure  => present,
            }
        }
        # Set profile scripts for custom tbb location
        file { "/etc/profile.d/50-maverick-tbb-cpath.sh":
            mode        => "644",
            owner       => "root",
            group       => "root",
            content     => 'NEWPATH="/srv/maverick/software/tbb/include"; if [ -n "${CPATH##*${NEWPATH}}" -a -n "${CPATH##*${NEWPATH}:*}" ]; then export CPATH=$NEWPATH:$CPATH; fi',
        } ->
        file { "/etc/profile.d/50-maverick-tbb-librarypath.sh":
            mode        => "644",
            owner       => "root",
            group       => "root",
            content     => 'NEWPATH="/srv/maverick/software/tbb/lib"; if [ -n "${LIBRARY_PATH##*${NEWPATH}}" -a -n "${LIBRARY_PATH##*${NEWPATH}:*}" ]; then export LIBRARY_PATH=$NEWPATH:$LIBRARY_PATH; fi',
        } ->
        file { "/etc/profile.d/50-maverick-tbb-ldlibrarypath.sh":
            mode        => "644",
            owner       => "root",
            group       => "root",
            content     => 'NEWPATH="/srv/maverick/software/tbb/lib"; if [ -n "${LD_LIBRARY_PATH##*${NEWPATH}}" -a -n "${LD_LIBRARY_PATH##*${NEWPATH}:*}" ]; then export LD_LIBRARY_PATH=$NEWPATH:$LD_LIBRARY_PATH; fi',
        }
        file { "/etc/profile.d/50-maverick-tbb-paths.sh":
            ensure      => absent,
        }

    }

}
