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
    #String $tbb_version = "v2021.3.0",
    String $tbb_version = "master",
    Boolean $openblas = true,
    String $openblas_version = "v0.3.17",
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
        # If raspberry, set atomic linker flag
        if $raspberry_present == "yes" {
            $_RPIFLAGS = "-DCMAKE_CXX_FLAGS=-latomic"
        } else {
            $_RPIFLAGS = ""
        }
        if ! ("install_flag_tbb" in $installflags) {
            oncevcsrepo { "git-tbb":
                gitsource   => "https://github.com/oneapi-src/oneTBB",
                dest        => "/srv/maverick/var/build/tbb",
                revision    => $tbb_version,
            } ->
            file { "/srv/maverick/var/build/tbb/build":
                ensure => directory,
                owner  => "mav",
                group  => "mav",
            } ->
            exec { "tbb-cmake":
                command => "/usr/bin/cmake ${_RPIFLAGS} -DPYTHON_EXECUTABLE=/srv/maverick/software/python/bin/python3 -DTBB_TEST=OFF -DTBB4PY_BUILD=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/tbb .. >/srv/maverick/var/log/build/tbb.cmake.log 2>&1",
                cwd     => "/srv/maverick/var/build/tbb/build",
                timeout => 0,
                user    => "mav",
                creates => "/srv/maverick/var/build/tbb/build/CMakeCache.txt",
            } ->
            exec { "tbb-build":
                command => "/usr/bin/cmake --build . --target all --config Release >/srv/maverick/var/log/build/tbb.build.log 2>&1",
                cwd     => "/srv/maverick/var/build/tbb/build",
                timeout => 0,
                user    => "mav",
                creates => "/srv/maverick/var/build/tbb/build/src/tbb/CMakeFiles/tbb.dir/version.cpp.o",
            } ->
            exec { "tbb-install":
                command => "/usr/bin/cmake ${_RPIFLAGS} -P cmake_install.cmake >/srv/maverick/var/log/build/tbb.install.log 2>&1",
                cwd     => "/srv/maverick/var/build/tbb/build",
                timeout => 0,
                user    => "mav",
                creates => "/srv/maverick/software/tbb/lib/libtbb.so.12",
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
            content     => 'NEWPATH="/srv/maverick/software/tbb/include"; export CPATH=${CPATH:-${NEWPATH}}; if [ -n "${CPATH##*${NEWPATH}}" -a -n "${CPATH##*${NEWPATH}:*}" ]; then export CPATH=$NEWPATH:$CPATH; fi',
        } ->
        file { "/etc/profile.d/50-maverick-tbb-librarypath.sh":
            mode        => "644",
            owner       => "root",
            group       => "root",
            content     => 'NEWPATH="/srv/maverick/software/tbb/lib"; export LIBRARY_PATH=${LIBRARY_PATH:-${NEWPATH}}; if [ -n "${LIBRARY_PATH##*${NEWPATH}}" -a -n "${LIBRARY_PATH##*${NEWPATH}:*}" ]; then export LIBRARY_PATH=$NEWPATH:$LIBRARY_PATH; fi',
        } ->
        file { "/etc/profile.d/50-maverick-tbb-ldlibrarypath.sh":
            mode        => "644",
            owner       => "root",
            group       => "root",
            content     => 'NEWPATH="/srv/maverick/software/tbb/lib"; export LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-${NEWPATH}}; if [ -n "${LD_LIBRARY_PATH##*${NEWPATH}}" -a -n "${LD_LIBRARY_PATH##*${NEWPATH}:*}" ]; then export LD_LIBRARY_PATH=$NEWPATH:$LD_LIBRARY_PATH; fi',
        }
        file { "/etc/profile.d/50-maverick-tbb-paths.sh":
            ensure      => absent,
        }

    }

}
