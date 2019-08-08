class maverick_vision::opencv (
    $contrib = true,
    $opencv_version = "4.1.1",
    $release = "Release", # Release or Debug, OpenCV build type
    $precompile_headers = false,
    $armv7l_optimize = false,
    $opencv_dldt_version = "2019",
    $openvino = false,
) {
    
    # Ensure gstreamer, tbb and openblas resources are applied before this class
    require maverick_vision::gstreamer
    require maverick_vision::visionlibs
    
    # Install dependencies
    if $operatingsystem == "Ubuntu" and ($operatingsystemrelease == "17.04" or $operatingsystemrelease == "17.10" or $operatingsystemrelease == "18.04") {
        ensure_packages(["libpng-dev"])
    } elsif $operatingsystem == "Ubuntu" and ($operatingsystemrelease == "16.04" or $operatingsystemrelease == "16.10") {
        ensure_packages(["libjasper-dev", "libpng12-dev"])
    }
    ensure_packages(["libjpeg-dev", "libtiff5-dev", "libgdal-dev", "libavcodec-dev", "libavformat-dev", "libswscale-dev", "libv4l-dev", "libxvidcore-dev", "libatlas-base-dev", "gfortran", "libeigen3-dev", "libavresample-dev", "libopenblas-dev", "libgdcm2-dev", "liblapacke-dev", "libgtk2.0-dev"])
    ensure_packages(["libglew-dev"])
    ensure_packages(["qt5-default"])

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
        
        if $maverick_vision::visionlibs::tbb == true {
            $_tbbopts = "-DCMAKE_INSTALL_RPATH=/srv/maverick/software/tbb/lib -DWITH_TBB=ON -DBUILD_TBB=OFF"
        } else {
            $_tbbopts = "-DWITH_TBB=OFF -DBUILD_TBB=OFF"
        }

        if $tegra_present == "yes" {
            if $tegra_model == "TX1" {
                $_tegra_arch = "-D CUDA_ARCH_BIN=5.3"
            } elsif $tegra_model == "TX2" {
                $_tegra_arch = "-D CUDA_ARCH_BIN=6.2"
            } else {
                $_tegra_arch = ""
            }

            # Patch OpenGL headers
            file { "/var/tmp/OpenGLHeader.patch":
                source      => "puppet:///modules/maverick_vision/OpenGLHeader.patch",
                owner       => "mav",
                group       => "mav",
            } ->
            exec { "nvidia-patch-glheaders":
                cwd     => "/usr/local/cuda/include",
                command => "/usr/bin/patch -N cuda_gl_interop.h /var/tmp/OpenGLHeader.patch",
                unless  => "/bin/grep '//#ifndef GL_VERSION' /usr/local/cuda/include/cuda_gl_interop.h",
            }
        }

        if $::hardwaremodel == "x86_64" {
            $_command           = "/usr/bin/cmake -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/opencv -DOPENCV_GENERATE_PKGCONFIG=YES -DCMAKE_BUILD_TYPE=${release} ${_pchstr} -DOPENCV_ENABLE_NONFREE=ON -DINSTALL_C_EXAMPLES=OFF -DPYTHON3_EXECUTABLE=/srv/maverick/software/python/bin/python3 -DINSTALL_PYTHON_EXAMPLES=ON ${contribstr} -DBUILD_EXAMPLES=ON -DWITH_LIBV4L=OFF -DWITH_EIGEN=ON -DWITH_OPENGL=ON -DENABLE_OPENGL=ON -DWITH_IPP=ON -DWITH_OPENVX=ON -DWITH_INTELPERC=ON -DWITH_VA=ON -DWITH_VA_INTEL=ON -DWITH_GSTREAMER=ON -DWITH_QT=OFF -DWITH_OPENNI2=ON -DENABLE_FAST_MATH=ON ${_tbbopts} .. >/srv/maverick/var/log/build/opencv.cmake.out 2>&1"
        } elsif $tegra_present == "yes" {
            $_command           = "/usr/bin/cmake -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/opencv -DOPENCV_GENERATE_PKGCONFIG=YES -DCMAKE_BUILD_TYPE=${release} ${_pchstr} -DOPENCV_ENABLE_NONFREE=ON -DINSTALL_C_EXAMPLES=OFF -DPYTHON3_EXECUTABLE=/srv/maverick/software/python/bin/python3 -DINSTALL_PYTHON_EXAMPLES=ON ${contribstr} -DBUILD_EXAMPLES=ON -DWITH_LIBV4L=ON -DWITH_EIGEN=ON -DWITH_OPENGL=ON -DENABLE_OPENGL=ON -DWITH_GSTREAMER=ON -DWITH_QT=ON -DWITH_OPENNI2=ON -DWITH_CUDA=ON -DCUDA_FAST_MATH=ON -DWITH_CUBLAS=ON ${_tbbopts} ${_tegra_arch} .. >/srv/maverick/var/log/build/opencv.cmake.out 2>&1"
        } else {
            $_command           = "/usr/bin/cmake -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/opencv -DOPENCV_GENERATE_PKGCONFIG=YES -DCMAKE_BUILD_TYPE=${release} ${_pchstr} -DOPENCV_ENABLE_NONFREE=ON -DINSTALL_C_EXAMPLES=OFF -DPYTHON3_EXECUTABLE=/srv/maverick/software/python/bin/python3 -DINSTALL_PYTHON_EXAMPLES=ON ${contribstr} -DBUILD_EXAMPLES=ON -DWITH_LIBV4L=OFF -DWITH_EIGEN=ON -DWITH_OPENGL=ON -DENABLE_OPENGL=ON -DWITH_GSTREAMER=ON -DWITH_QT=OFF -DWITH_OPENNI2=ON ${_armopts} ${_tbbopts} .. >/srv/maverick/var/log/build/opencv.cmake.out 2>&1"
        }
    
        if Numeric($memorysize_mb) < 1000 {
            $_makej = 1
        } elsif Numeric($memorysize_mb) < 2000 {
            $_makej = 2
        } else {
            $_makej = Numeric($processorcount)
        }
        exec { "opencv-prepbuild":
            user        => "mav",
            timeout     => 0,
            environment => [
                "PKG_CONFIG_PATH=/srv/maverick/software/gstreamer/lib/pkgconfig",
                "TBB_LIB_DIR=/srv/maverick/software/tbb/lib",
                "LIBRARY_PATH=/srv/maverick/software/tbb/lib:/srv/maverick/software/gstreamer/lib",
                "LD_LIBRARY_PATH=/srv/maverick/software/tbb/lib:/srv/maverick/software/gstreamer/lib",
                "TBBROOT=/srv/maverick/software/tbb",
                "CPATH=/srv/maverick/software/tbb/include",
                "OpenBLAS_HOME=/srv/maverick/software/openblas",
            ],
            command     => $_command,
            cwd         => "/srv/maverick/var/build/opencv/build",
            creates     => "/srv/maverick/var/build/opencv/build/Makefile",
            require     => [ Class["maverick_vision::visionlibs"], Class["maverick_vision::gstreamer"], File["/srv/maverick/var/build/opencv/build"], Package["libjpeg-dev", "libtiff5-dev", "libavcodec-dev", "libavformat-dev", "libswscale-dev", "libv4l-dev", "libxvidcore-dev", "libatlas-base-dev", "gfortran", "libeigen3-dev", "libavresample-dev", "libopenblas-dev", "libgdal-dev", "libgdcm2-dev", "liblapacke-dev", "libgtk2.0-dev", "qt5-default"] ], # ensure we have all the dependencies satisfied
        } ->
        exec { "opencv-build":
            user        => "mav",
            timeout     => 0,
            environment => [
                "LIBRARY_PATH=/srv/maverick/software/tbb/lib:/srv/maverick/software/gstreamer/lib",
                "LD_LIBRARY_PATH=/srv/maverick/software/tbb/lib:/srv/maverick/software/gstreamer/lib",
                "CPATH=/srv/maverick/software/tbb/include",
            ],
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
        #content     => "export PATH=/srv/maverick/software/opencv/bin:\$PATH",
        content     => 'NEWPATH="/srv/maverick/software/opencv/bin"; if [ -n "${PATH##*${NEWPATH}}" -a -n "${PATH##*${NEWPATH}:*}" ]; then export PATH=$NEWPATH:$PATH; fi',
    } ->
    file { "/etc/profile.d/40-maverick-opencv-pkgconfig.sh":
        mode        => "644",
        owner       => "root",
        group       => "root",
        #content     => "export PKG_CONFIG_PATH=/srv/maverick/software/opencv/lib/pkgconfig:\$PKG_CONFIG_PATH",
        content     => 'NEWPATH="/srv/maverick/software/opencv/lib/pkgconfig"; if [ -n "${PKG_CONFIG_PATH##*${NEWPATH}}" -a -n "${PKG_CONFIG_PATH##*${NEWPATH}:*}" ]; then export PKG_CONFIG_PATH=$NEWPATH:$PKG_CONFIG_PATH; fi',
    } ->
    file { "/etc/profile.d/40-maverick-opencv-ldlibrarypath.sh":
        mode        => "644",
        owner       => "root",
        group       => "root",
        content     => 'NEWPATH="/srv/maverick/software/opencv/lib"; if [ -n "${LD_LIBRARY_PATH##*${NEWPATH}}" -a -n "${LD_LIBRARY_PATH##*${NEWPATH}:*}" ]; then export LD_LIBRARY_PATH=$NEWPATH:$LD_LIBRARY_PATH; fi',
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
        #content     => "export PYTHONPATH=\$PYTHONPATH:/srv/maverick/software/opencv/lib/python3.7/site-packages",
        content     => 'NEWPATH="/srv/maverick/software/opencv/lib/python3.7/site-packages"; if [ -n "${PYTHONPATH##*${NEWPATH}}" -a -n "${PYTHONPATH##*${NEWPATH}:*}" ]; then export PYTHONPATH=$NEWPATH:$PYTHONPATH; fi',
    } ->
    file { "/etc/profile.d/40-maverick-opencv-cmake.sh":
        mode        => "644",
        owner       => "root",
        group       => "root",
        content     => 'NEWPATH="/srv/maverick/software/opencv"; if [ -n "${CMAKE_PREFIX_PATH##*${NEWPATH}}" -a -n "${CMAKE_PREFIX_PATH##*${NEWPATH}:*}" ]; then export CMAKE_PREFIX_PATH=$NEWPATH:$CMAKE_PREFIX_PATH; fi',
    }

    if $openvino == true {
        # If ~/var/build/.install_flag_opencv_dldt exists, skip pulling source and compiling
        if ! ("install_flag_opencv_dldt" in $installflags) {
            oncevcsrepo { "git-opencv_dldt":
                gitsource   => "https://github.com/opencv/dldt.git",
                dest        => "/srv/maverick/var/build/opencv_dldt",
                revision    => $opencv_dldt_version,
            } ->
            # Create build directory
            file { "/srv/maverick/var/build/opencv_dldt/inference-engine/build":
                ensure      => directory,
                owner       => "mav",
                group       => "mav",
                mode        => "755",
            } ->
            exec { "opencv_submodules-init":
                command     => "/usr/bin/git submodule init",
                cwd         => "/srv/maverick/var/build/opencv_dldt/inference-engine",
                creates     => "/srv/maverick/var/build/opencv_dldt/inference-engine/thirdparty/ade",
                user        => "mav",
            } ->
            exec { "opencv_submodules-update":
                command     => "/usr/bin/git submodule update --recursive",
                cwd         => "/srv/maverick/var/build/opencv_dldt/inference-engine",
                creates     => "/srv/maverick/var/build/opencv_dldt/inference-engine/thirdparty/ade/CMakeLists.txt",
                user        => "mav",
            } ->
            exec { "opencv_dldt-prepbuild":
                user        => "mav",
                timeout     => 0,
                command     => "/usr/bin/cmake -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/opencv_dldt -DCMAKE_BUILD_TYPE=Release .. >/srv/maverick/var/log/build/opencv_dldt.prepbuild.out 2>&1",
                cwd         => "/srv/maverick/var/build/opencv_dldt/inference-engine/build",
                creates     => "/srv/maverick/var/build/opencv_dldt/inference-engine/build/Makefile",
                require     => Exec["opencv-prepbuild"],
            } ->
            exec { "opencv_dldt-build":
                user        => "mav",
                timeout     => 0,
                command     => "/usr/bin/make >/srv/maverick/var/log/build/opencv_dldt.build.out 2>&1",
                cwd         => "/srv/maverick/var/build/opencv_dldt/inference-engine/build",
                #creates     => ,
                #require     => Exec["opencv-prepbuild"],
            } ->
            exec { "opencv_dldt-install":
                user        => "mav",
                timeout     => 0,
                command     => "/usr/bin/make install >/srv/maverick/var/log/build/opencv_dldt.install.out 2>&1",
                cwd         => "/srv/maverick/var/build/opencv_dldt/inference-engine/build",
                #creates     => $_install_creates,
            } ->
            file { "/srv/maverick/var/build/.install_flag_opencv_dldt":
                ensure      => present,
            }
        }
    }

}
