class maverick_hardware::peripheral::caspa (
) {
    
    ensure_packages(["libjpeg62"])
    file { ["/srv/maverick/var/build/caspa", "/srv/maverick/var/build/caspa/gtest"]:
        owner       => "mav",
        group       => "mav",
        mode        => "755",
        ensure      => directory,
    } ->
    oncevcsrepo { "git-icamerasrc":
        gitsource   => "https://github.com/01org/icamerasrc.git",
        dest        => "/srv/maverick/var/build/caspa/icamerasrc",
        revision    => "8a4a9e575b382ff70f1950a4b52c9d835d982101",
        depth       => "",
    } ->
    oncevcsrepo { "git-intel-camera-dev-support":
        gitsource   => "https://github.com/Intel-5xx-Camera/intel-camera-dev-support",
        dest        => "/srv/maverick/var/build/caspa/intel-camera-dev-support",
    } ->
    oncevcsrepo { "git-intel-camera-adaptation":
        gitsource   => "https://github.com/Intel-5xx-Camera/intel-camera-adaptation",
        dest        => "/srv/maverick/var/build/caspa/intel-camera-adaptation",
    } ->
    oncevcsrepo { "git-intel-camera-drivers":
        gitsource   => "https://github.com/01org/intel-camera-drivers",
        dest        => "/srv/maverick/var/build/caspa/intel-camera-drivers",
    } ->
    oncevcsrepo { "git-meta-intel-camera":
        gitsource   => "https://github.com/01org/meta-intel-camera",
        dest        => "/srv/maverick/var/build/caspa/meta-intel-cameras",
    } ->
    exec { "compile-gtest":
        command     => "/usr/bin/cmake -DCMAKE_BUILD_TYPE=RELEASE /usr/src/gtest && make && sudo cp libg* /usr/local/lib",
        cwd         => "/srv/maverick/var/build/caspa/gtest",
        creates     => "/usr/local/lib/libgtest_main.a",
    } ->
    exec { "compile-camera-dev-support":
        command     => "/usr/bin/autoreconf -i && ./configure && make && make install",
        cwd         => "/srv/maverick/var/build/caspa/intel-camera-dev-support",
        creates     => "/usr/local/lib/libicamera_adapter.so.0.0.0",
    } ->
    exec { "compile-camera-dev-support-adapter":
        command     => "/usr/bin/autoreconf -i && ./configure && make && make install",
        cwd         => "/srv/maverick/var/build/caspa/intel-camera-dev-support/icamera_adapter",
        creates     => "/usr/local/lib/libcutils.so",
    } ->
    exec { "compile-icamerasrc":
        command     => "/usr/bin/autoreconf -i && /srv/maverick/var/build/caspa/icamerasrc/configure --with-pkg-config-path=/srv/maverick/software/gstreamer/lib/pkgconfig --prefix=/srv/maverick/software/gstreamer --with-headercheck=no --with-libcheck=no --with-cameralib=licamera_adapter --with-androidstubs=yes && make && make install",
        cwd         => "/srv/maverick/var/build/caspa/icamerasrc",
        creates     => "/srv/maverick/software/gstreamer/lib/gstreamer-1.0/libgsticamerasrc.so",
        user        => "mav",
        require     => Class["maverick_vision::gstreamer"],
    }
}