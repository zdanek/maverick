class maverick_vision::opencv (
    $contrib = false,
) {
    
    # Compile opencv3, note this can take a while.
    # Note that we're deliberately not installing into either sitl or fc python virtualenvs as we want to access from both,
    #  so we install into global.
    
    # Install dependencies
    ensure_packages(["libjpeg-dev", "libtiff5-dev", "libjasper-dev", "libpng12-dev"])
    ensure_packages(["libavcodec-dev", "libavformat-dev", "libswscale-dev", "libv4l-dev", "libxvidcore-dev", "libx264-dev"])
    ensure_packages(["libatlas-base-dev", "gfortran"])
    ensure_packages(["python2.7-dev", "libpython3-all-dev"])
    ensure_packages(["libgtk2.0-dev"])
    ensure_packages(["libtbb-dev", "libeigen3-dev"])
    ensure_packages(["libvtk6-dev", "vtk6"])
    
    # Pull opencv and opencv_contrib from git
    oncevcsrepo { "git-opencv":
        gitsource   => "https://github.com/Itseez/opencv.git",
        dest        => "/srv/maverick/var/build/opencv",
        # revision    => "3.1.0",
    } ->
    # Create build directory
    file { "/srv/maverick/var/build/opencv/build":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => 755,
    } ->
    # Run cmake and generate build files
    exec { "opencv-prepbuild":
        user        => "mav",
        timeout     => 0,
        # command     => "/usr/bin/cmake -D CMAKE_INSTALL_PREFIX=/srv/maverick/software/opencv -D CMAKE_BUILD_TYPE=RELEASE -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_EIGEN=ON -DWITH_OPENGL=ON -DENABLE_NEON=ON -DWITH_TBB=ON -DBUILD_TBB=ON -DENABLE_VFPV3=ON .. >/srv/maverick/var/log/build/opencv.cmake.out 2>&1",
        command     => "/usr/bin/cmake -D CMAKE_INSTALL_PREFIX=/srv/maverick/software/opencv -D CMAKE_BUILD_TYPE=RELEASE -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_EIGEN=ON -DWITH_OPENGL=ON -DENABLE_NEON=ON -DWITH_TBB=OFF -DBUILD_TBB=OFF -DENABLE_VFPV3=ON .. >/srv/maverick/var/log/build/opencv.cmake.out 2>&1",
        cwd         => "/srv/maverick/var/build/opencv/build",
        creates     => "/srv/maverick/var/build/opencv/build/Makefile",
        require     => Package["libeigen3-dev", "libjpeg-dev", "libtiff5-dev", "libjasper-dev", "libpng12-dev", "libavcodec-dev", "libavformat-dev", "libswscale-dev", "libv4l-dev", "libxvidcore-dev", "libx264-dev", "libatlas-base-dev", "gfortran", "libgtk2.0-dev", "python2.7-dev", "libpython3-all-dev", "python-numpy", "python3-numpy"] # ensure we have all the dependencies satisfied
    } ->
    exec { "opencv-build":
        user        => "mav",
        timeout     => 0,
        command     => "/usr/bin/make -j${::processorcount} >/srv/maverick/var/log/build/opencv.build.out 2>&1",
        cwd         => "/srv/maverick/var/build/opencv/build",
        creates     => "/srv/maverick/var/build/opencv/build/lib/cv2.so",
    } ->
    #exec { "opencv-fixbindeb":
    #    user        => "mav",
    #    command     => "/bin/sed -i -e 's/CPACK_BINARY_DEB:BOOL=OFF/CPACK_BINARY_DEB:BOOL=ON/' /srv/maverick/var/build/opencv/build/CMakeCache.txt",
    #    unless      => "/bin/grep 'CPACK_BINARY_DEB:BOOL=ON' /srv/maverick/var/build/opencv/build/CMakeCache.txt",
    #} ->
    exec { "opencv-install":
        user        => "mav",
        timeout     => 0,
        command     => "/usr/bin/make install >/srv/maverick/var/log/build/opencv.install.out 2>&1",
        cwd         => "/srv/maverick/var/build/opencv/build",
        creates     => "/srv/maverick/software/opencv/lib/libopencv_core.so",
    }
    file { "/etc/profile.d/maverick-opencv.sh":
        mode        => 644,
        owner       => "root",
        group       => "root",
        content     => "PKG_CONFIG_PATH=/srv/maverick/software/opencv/lib/pkgconfig:\$PKG_CONFIG_PATH; LD_LIBRARY_PATH=/srv/maverick/software/opencv/lib:\$LD_LIBRARY_PATH",
    }
    #exec { "opencv-package":
    #    user        => "mav",
    #    timeout     => 0,
    #    command     => "/usr/bin/make package >/srv/maverick/var/log/build/opencv.package.out 2>&1",
    #    cwd         => "/srv/maverick/var/build/opencv/build",
    #    # creates     => "/usr/local/lib/libopencv_core.so",
    #}
    
    if $contrib == true {
        oncevcsrepo { "git-opencv_contrib":
            gitsource   => "https://github.com/Itseez/opencv_contrib.git",
            dest        => "/srv/maverick/var/build/opencv_contrib",
        } ->
        # Run cmake and generate build files
        exec { "opencv-contrib-prepbuild":
            user        => "mav",
            timeout     => 0,
            command     => "/usr/bin/cmake -D CMAKE_INSTALL_PREFIX=/srv/maverick/software/opencv -D CMAKE_BUILD_TYPE=RELEASE -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D OPENCV_EXTRA_MODULES_PATH=/srv/maverick/var/build/opencv_contrib/modules -DBUILD_opencv_legacy=OFF -D BUILD_EXAMPLES=ON -D BUILD_PACKAGE=ON -D WITH_EIGEN=ON -D WITH_OPENGL=ON -D ENABLE_NEON=ON -D WITH_TBB=OFF -D BUILD_TBB=OFF -D ENABLE_VFPV3=ON .. >/srv/maverick/var/log/build/opencv.contrib.cmake.out 2>&1",
            cwd         => "/srv/maverick/var/build/opencv/build",
            unless      => "/bin/grep contrib verison_string.tmp",
            require     => Exec["opencv-install"],
        } ->
        exec { "opencv-contrib-build":
            user        => "mav",
            timeout     => 0,
            command     => "/usr/bin/make -j${::processorcount} >/srv/maverick/var/log/build/opencv.contrib.build.out 2>&1",
            cwd         => "/srv/maverick/var/build/opencv/build",
            creates     => "/srv/maverick/var/build/opencv/build/lib/libopencv_structured_light.so",
        } ->
        exec { "opencv-contrib-install":
            user        => "mav",
            timeout     => 0,
            command     => "/usr/bin/make install >/srv/maverick/var/log/build/opencv.contrib.install.out 2>&1",
            cwd         => "/srv/maverick/var/build/opencv/build",
            creates     => "/srv/maverick/software/opencv/lib/libopencv_structured_light.so",
        }
    }
    
}