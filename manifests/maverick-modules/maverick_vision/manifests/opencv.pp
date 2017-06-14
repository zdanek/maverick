class maverick_vision::opencv (
    $contrib = true,
    $opencv_version = "3.2.0",
    $release = "Release", # Release or Debug, OpenCV build type
    $precompile_headers = false,
    $armv7l_optimize = false,
) {
    
    # Ensure gstreamer resources are applied before this class
    require maverick_vision::gstreamer
    
    # Compile opencv3, note this can take a while.
    # Note that we're deliberately not installing the python bindings into either sitl or fc python virtualenvs
    # as we want to access from both, so we install into global.
    
    # Install dependencies
    if $operatingsystem == "Ubuntu" and ($operatingsystemrelease == "17.04" or $operatingsystemrelease == "17.10") {
        ensure_packages(["libpng-dev"])
    } elsif $operatingsystem == "Ubuntu" and ($operatingsystemrelease == "16.04" or $operatingsystemrelease == "16.10") {
        ensure_packages(["libjasper-dev", "libpng12-dev"])
    }
    ensure_packages(["libjpeg-dev", "libtiff5-dev", "libgdal-dev", "libavcodec-dev", "libavformat-dev", "libswscale-dev", "libv4l-dev", "libxvidcore-dev", "libatlas-base-dev", "gfortran", "libeigen3-dev", "libavresample-dev", "libopenblas-dev", "libgdcm2-dev", "liblapacke-dev", "libgtk2.0-dev"])
    ensure_packages(["python2.7-dev", "libpython3-all-dev"])
    ensure_packages(["libgtk2.0-dev"])
    # ensure_packages(["qtbase5-dev"])
    if $architecture == "amd64" { # https://github.com/fnoop/maverick/issues/233
        ensure_packages(["libtbb-dev", "libtbb2"])
    }
    
    # If ~/var/build/.install_flag_opencv exists, skip pulling source and compiling
    if ! ("install_flag_opencv" in $installflags) {
        # Pull opencv and opencv_contrib from git
        oncevcsrepo { "git-opencv":
            gitsource   => "https://github.com/opencv/opencv.git",
            dest        => "/srv/maverick/var/build/opencv",
            revision    => $opencv_version,
        } ->
        # Create build directory
        file { "/srv/maverick/var/build/opencv/build":
            ensure      => directory,
            owner       => "mav",
            group       => "mav",
            mode        => "755",
        }
        
        # Run cmake and generate build files
        if $contrib == true {
            oncevcsrepo { "git-opencv_contrib":
                gitsource   => "https://github.com/opencv/opencv_contrib.git",
                dest        => "/srv/maverick/var/build/opencv_contrib",
                revision    => $opencv_version,
                before      => Oncevcsrepo["git-opencv"],
            }
            $contribstr = "-DOPENCV_EXTRA_MODULES_PATH=/srv/maverick/var/build/opencv_contrib/modules -DBUILD_opencv_legacy=OFF"
            $_creates           = "/srv/maverick/var/build/opencv/build/lib/libopencv_structured_light.so"
            $_install_creates   = "/srv/maverick/software/opencv/lib/libopencv_structured_light.so"
        } else {
            $contribstr = ""
            $_creates           = "/srv/maverick/var/build/opencv/build/lib/libopencv_highgui.so"
            $_install_creates   = "/srv/maverick/software/opencv/lib/libopencv_highgui.so"
        }
        
        # Turn off precompiling headers by default, as it uses a lot of space
        if $precompile_headers == false {
            $_pchstr = "-DENABLE_PRECOMPILED_HEADERS=NO"
        } else {
            $_pchstr = ""
        }
    
        if $armv7l_optimize == true {
            $_armopts = "-DENABLE_NEON=ON -DENABLE_VFPV3=ON"
        } else {
            $_armopts = "-DENABLE_NEON=OFF -DENABLE_VFPV3=OFF"
        }
        
        if $architecture == "amd64" {
            # $_command           = "/usr/bin/cmake -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/opencv -DCMAKE_BUILD_TYPE=${release} ${_pchstr} -DINSTALL_C_EXAMPLES=ON -DINSTALL_PYTHON_EXAMPLES=ON ${contribstr} -DBUILD_EXAMPLES=ON -DWITH_LIBV4L=OFF -DWITH_EIGEN=ON -DWITH_OPENGL=ON -DENABLE_OPENGL=ON -DENABLE_NEON=ON -DWITH_TBB=ON -DBUILD_TBB=ON -DENABLE_VFPV3=ON -DWITH_IPP=ON -DWITH_OPENVX=ON -DWITH_INTELPERC=ON -DWITH_VA=ON -DWITH_VA_INTEL=ON -DWITH_GSTREAMER=ON -DWITH_QT=OFF -DWITH_OPENNI2=ON -DENABLE_FAST_MATH=ON -DENABLE_SSE41=ON -DENABLE_SSE42=ON -DOPENVX_ROOT=/opt/intel/computer_vision_sdk_2017.0.087/openvx -DWITH_OPENVX=YES .. >/srv/maverick/var/log/build/opencv.cmake.out 2>&1"
            $_command           = "/usr/bin/cmake -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/opencv -DCMAKE_BUILD_TYPE=${release} ${_pchstr} -DINSTALL_C_EXAMPLES=ON -DINSTALL_PYTHON_EXAMPLES=ON ${contribstr} -DBUILD_EXAMPLES=ON -DWITH_LIBV4L=OFF -DWITH_EIGEN=ON -DWITH_OPENGL=ON -DENABLE_OPENGL=ON -DENABLE_NEON=ON -DWITH_TBB=ON -DBUILD_TBB=ON -DENABLE_VFPV3=ON -DWITH_IPP=ON -DWITH_OPENVX=ON -DWITH_INTELPERC=ON -DWITH_VA=ON -DWITH_VA_INTEL=ON -DWITH_GSTREAMER=ON -DWITH_QT=OFF -DWITH_OPENNI2=ON -DENABLE_FAST_MATH=ON -DENABLE_SSE41=ON -DENABLE_SSE42=ON .. >/srv/maverick/var/log/build/opencv.cmake.out 2>&1"
        } else {
            $_command           = "/usr/bin/cmake -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/opencv -DCMAKE_BUILD_TYPE=${release} ${_pchstr} -DINSTALL_C_EXAMPLES=ON -DINSTALL_PYTHON_EXAMPLES=ON ${contribstr} -DBUILD_EXAMPLES=ON -DWITH_LIBV4L=OFF -DWITH_EIGEN=ON -DWITH_OPENGL=ON -DENABLE_OPENGL=ON -DWITH_TBB=OFF -DBUILD_TBB=OFF -DWITH_GSTREAMER=ON -DWITH_QT=OFF -DWITH_OPENNI2=ON ${_armopts} .. >/srv/maverick/var/log/build/opencv.cmake.out 2>&1"
        }
    
        if $raspberry_present == yes {
            $_makej = 2
        } else {
            $_makej = $processorcount
        }
        exec { "opencv-prepbuild":
            user        => "mav",
            timeout     => 0,
            environment => ["PKG_CONFIG_PATH=/srv/maverick/software/gstreamer/lib/pkgconfig"],
            command     => $_command,
            cwd         => "/srv/maverick/var/build/opencv/build",
            creates     => "/srv/maverick/var/build/opencv/build/Makefile",
            require     => [ Class["maverick_vision::gstreamer"], File["/srv/maverick/var/build/opencv/build"], Package["libjpeg-dev", "libtiff5-dev", "libavcodec-dev", "libavformat-dev", "libswscale-dev", "libv4l-dev", "libxvidcore-dev", "libatlas-base-dev", "gfortran", "libeigen3-dev", "libavresample-dev", "libopenblas-dev", "libgdal-dev", "libgdcm2-dev", "liblapacke-dev", "libgtk2.0-dev", "python2.7-dev", "libpython3-all-dev"] ], # ensure we have all the dependencies satisfied
        } ->
        exec { "opencv-build":
            user        => "mav",
            timeout     => 0,
            command     => "/usr/bin/make -j${_makej} >/srv/maverick/var/log/build/opencv.build.out 2>&1",
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
        } ->
        file { "/srv/maverick/var/build/.install_flag_opencv":
            ensure      => present,
        }
    } else {
        # If we don't build opencv, we still need something for other manifest dependencies
        file { "/srv/maverick/var/build/.install_flag_opencv":
            ensure      => present,
        }
    }
    
    file { "/etc/profile.d/40-maverick-opencv-path.sh":
        mode        => "644",
        owner       => "root",
        group       => "root",
        content     => "export PATH=/srv/maverick/software/opencv/bin:\$PATH",
    } ->
    file { "/etc/profile.d/40-maverick-opencv-pkgconfig.sh":
        mode        => "644",
        owner       => "root",
        group       => "root",
        content     => "export PKG_CONFIG_PATH=/srv/maverick/software/opencv/lib/pkgconfig:\$PKG_CONFIG_PATH",
    } ->
    file { "/etc/profile.d/40-maverick-opencv-ldlibrarypath.sh":
        ensure      => absent,
    } ->
    file { "/etc/ld.so.conf.d/maverick-opencv.conf":
        mode        => "644",
        owner       => "root",
        group       => "root",
        content     => "/srv/maverick/software/opencv/lib",
        notify      => Exec["maverick-ldconfig"],
    } ->
    file { "/etc/profile.d/40-maverick-opencv-pythonpath.sh":
        mode        => "644",
        owner       => "root",
        group       => "root",
        content     => "export PYTHONPATH=/srv/maverick/software/opencv/lib/python2.7/dist-packages:\$PYTHONPATH",
    } ->
    file { "/etc/profile.d/40-maverick-opencv-cmake.sh":
        mode        => "644",
        owner       => "root",
        group       => "root",
        content     => "export CMAKE_PREFIX_PATH=/srv/maverick/software/opencv:\$CMAKE_PREFIX_PATH",
    }

}