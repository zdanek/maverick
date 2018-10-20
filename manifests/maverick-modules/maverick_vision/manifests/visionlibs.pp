class maverick_vision::visionlibs (
    $tbb = true,
    $tbb_version = "2019_U1",
    $openblas = true,
    $openblas_version = "v0.3.3",
) {
    
    if $openblas == true {
        # If ~/var/build/.install_flag_openblas exists, skip pulling source and compiling
        if ! ("install_flag_openblas" in $installflags) {
            oncevcsrepo { "git-opencv":
                gitsource   => "https://github.com/xianyi/OpenBLAS.git",
                dest        => "/srv/maverick/var/build/openblas",
                revision    => $openblas_version,
            } ->
            exec { "openblas-make":
                command => "/usr/bin/make",
                creates => "/srv/maverick/software/openblas",
                cwd     => "/srv/maverick/var/build/openblas/libopenblas.so",
                timeout => 0,
            } ->
            exec { "openblas-makeinstall":
                command => "/usr/bin/make install PREFIX=/srv/maverick/software/openblas",
                creates => "/srv/maverick/software/openblas",
                cwd     => "/srv/maverick/var/build/openblas/libopenblas.so",
                timeout => 0,
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
                gitsource   => "https://github.com/01org/tbb.git",
                dest        => "/srv/maverick/var/build/tbb",
                revision    => $tbb_version,
            } ->
            exec { "tbb-make":
                command => "/usr/bin/make",
                creates => "/srv/maverick/software/tbb",
                cwd     => "/srv/maverick/var/build/tbb",
                timeout => 0,
            } ->
            file { "/srv/maverick/software/tbb":
                ensure  => directory,
                owner   => "mav",
                group   => "mav",
            } ->
            file { "/srv/maverick/var/build/tbb/install.sh":
                source  => "puppet:///modules/maverick_vision/tbb_install.sh",
                mode    => "0755",
            } ->
            exec { "tbb-install":
                command => "/srv/maverick/var/build/tbb/install.sh",
                user    => "mav",
                cwd     => "/srv/maverick/var/build/tbb",
            } ->
            file { "/srv/maverick/var/build/.install_flag_tbb":
                ensure  => present,
            }
        }
    }

}