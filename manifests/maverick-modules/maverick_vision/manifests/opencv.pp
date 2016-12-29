class maverick_vision::opencv (
    $contrib = true,
) {
    
    # Compile opencv3, note this can take a while.
    # Note that we're deliberately not installing the python bindings into either sitl or fc python virtualenvs
    # as we want to access from both, so we install into global.
    
    # Install dependencies
    ensure_packages(["libjpeg-dev", "libtiff5-dev", "libjasper-dev", "libpng12-dev", "libavcodec-dev", "libavformat-dev", "libswscale-dev", "libv4l-dev", "libxvidcore-dev", "libx264-dev", "libatlas-base-dev", "gfortran", "libeigen3-dev"])
    ensure_packages(["python2.7-dev", "libpython3-all-dev"])
    ensure_packages(["libgtk2.0-dev"])
    ensure_packages(["libopenni2-dev"])
    # ensure_packages(["qtbase5-dev"])
    # ensure_packages(["libtbb-dev"]) # https://github.com/fnoop/maverick/issues/233
    
    # Pull opencv and opencv_contrib from git
    oncevcsrepo { "git-opencv":
        gitsource   => "https://github.com/Itseez/opencv.git",
        dest        => "/srv/maverick/var/build/opencv",
        revision    => "3.2.0",
    } ->
    oncevcsrepo { "git-opencv_contrib":
        gitsource   => "https://github.com/Itseez/opencv_contrib.git",
        dest        => "/srv/maverick/var/build/opencv_contrib",
        revision    => "3.2.0",
    } ->
    # Create build directory
    file { "/srv/maverick/var/build/opencv/build":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => 755,
    }
    
    # Run cmake and generate build files
    if $contrib == true {
        $_command           = "/usr/bin/cmake -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/opencv -DCMAKE_BUILD_TYPE=RELEASE -DINSTALL_C_EXAMPLES=ON -DINSTALL_PYTHON_EXAMPLES=ON -DOPENCV_EXTRA_MODULES_PATH=/srv/maverick/var/build/opencv_contrib/modules -DBUILD_opencv_legacy=OFF -DBUILD_EXAMPLES=ON -DWITH_EIGEN=ON -DWITH_OPENGL=ON -DENABLE_NEON=ON -DWITH_TBB=OFF -DBUILD_TBB=OFF -DENABLE_VFPV3=ON -DWITH_GSTREAMER=ON -DWITH_QT=OFF -DWITH_OPENNI2=ON .. >/srv/maverick/var/log/build/opencv.cmake.out 2>&1"
        $_creates           = "/srv/maverick/var/build/opencv/build/lib/libopencv_structured_light.so"
        $_install_creates   = "/srv/maverick/software/opencv/lib/libopencv_structured_light.so"
    } else {
        $_command           = "/usr/bin/cmake -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/opencv -DCMAKE_BUILD_TYPE=RELEASE -DINSTALL_C_EXAMPLES=ON -DINSTALL_PYTHON_EXAMPLES=ON -DBUILD_EXAMPLES=ON -DWITH_EIGEN=ON -DWITH_OPENGL=ON -DENABLE_NEON=ON -DWITH_TBB=OFF -DBUILD_TBB=OFF -DENABLE_VFPV3=ON -DWITH_GSTREAMER=ON -DWITH_QT=OFF -DWITH_OPENNI2=ON .. >/srv/maverick/var/log/build/opencv.cmake.out 2>&1"
        $_creates           = "/srv/maverick/var/build/opencv/build/lib/cv2.so"
        $_install_creates   = "/srv/maverick/software/opencv/lib/libopencv_core.so"
    }
    exec { "opencv-prepbuild":
        user        => "mav",
        timeout     => 0,
        environment => ["PKG_CONFIG_PATH=/srv/maverick/software/gstreamer/lib/pkgconfig"],
        command     => $_command,
        cwd         => "/srv/maverick/var/build/opencv/build",
        creates     => "/srv/maverick/var/build/opencv/build/Makefile",
        require     => [ Class["maverick_vision::gstreamer"], File["/srv/maverick/var/build/opencv/build"], Package["libeigen3-dev", "libjpeg-dev", "libtiff5-dev", "libjasper-dev", "libpng12-dev", "libavcodec-dev", "libavformat-dev", "libswscale-dev", "libv4l-dev", "libxvidcore-dev", "libx264-dev", "libatlas-base-dev", "gfortran", "libgtk2.0-dev", "python2.7-dev", "libpython3-all-dev", "python-numpy", "python3-numpy"] ], # ensure we have all the dependencies satisfied
    } ->
    exec { "opencv-build":
        user        => "mav",
        timeout     => 0,
        command     => "/usr/bin/make -j${::processorcount} >/srv/maverick/var/log/build/opencv.build.out 2>&1",
        cwd         => "/srv/maverick/var/build/opencv/build",
        creates     => $_creates,
        require     => Exec["opencv-prepbuild"],
    } ->
    exec { "opencv-install":
        user        => "mav",
        timeout     => 0,
        command     => "/usr/bin/make install >/srv/maverick/var/log/build/opencv.install.out 2>&1",
        cwd         => "/srv/maverick/var/build/opencv/build",
        creates     => $_install_creates,
    }
    file { "/etc/profile.d/maverick-opencv.sh":
        mode        => 644,
        owner       => "root",
        group       => "root",
        content     => "PKG_CONFIG_PATH=/srv/maverick/software/opencv/lib/pkgconfig:\$PKG_CONFIG_PATH; LD_LIBRARY_PATH=/srv/maverick/software/opencv/lib:\$LD_LIBRARY_PATH",
    }
    
}