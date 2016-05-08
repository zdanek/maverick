class maverick-vision::cv::init (
) {
    
    # Compile opencv3, note this can take a while.  This is (loosely) based on the guide here:
    #  http://www.pyimagesearch.com/2016/04/18/install-guide-raspberry-pi-3-raspbian-jessie-opencv-3/
    # Note that we're deliberately not installing into either sitl or fc virtualenvs as we want to access from both,
    #  so we install into global.
    
    # Install dependencies
    ensure_packages(["libjpeg-dev", "libtiff5-dev", "libjasper-dev", "libpng12-dev"])
    ensure_packages(["libavcodec-dev", "libavformat-dev", "libswscale-dev", "libv4l-dev", "libxvidcore-dev", "libx264-dev"])
    ensure_packages(["libatlas-base-dev", "gfortran"])
    ensure_packages(["python2.7-dev", "python3-dev"])
    ensure_packages(["libgstreamer-plugins-base1.0-dev"])
    
    # Pull opencv and opencv_contrib from git
    vcsrepo { "/srv/maverick/software/opencv":
        ensure		=> present,
        provider 	=> git,
        source		=> "https://github.com/Itseez/opencv.git",
        revision	=> "master",
        owner		=> "mav",
        group		=> "mav",
    } ->
    vcsrepo { "/srv/maverick/software/opencv_contrib":
        ensure		=> present,
        provider 	=> git,
        source		=> "https://github.com/Itseez/opencv_contrib.git",
        revision	=> "master",
        owner		=> "mav",
        group		=> "mav",
    } ->
    # Create build directory
    file { "/srv/maverick/software/opencv/build":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => 755,
    } ->
    # Run cmake and generate build files
    exec { "opencv-prepbuild":
        user        => "mav",
        timeout     => 0,
        #command     => "/usr/bin/cmake -j${::processorcount} -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D INSTALL_PYTHON_EXAMPLES=ON -D OPENCV_EXTRA_MODULES_PATH=/srv/maverick/software/opencv_contrib/modules -D BUILD_EXAMPLES=ON .. >/var/tmp/opencv.cmake.out 2>&1",
        command     => "/usr/bin/cmake -D CMAKE_BUILD_TYPE=RELEASE -D INSTALL_PYTHON_EXAMPLES=ON -D OPENCV_EXTRA_MODULES_PATH=/srv/maverick/software/opencv_contrib/modules -D BUILD_EXAMPLES=ON .. >/var/tmp/opencv.cmake.out 2>&1",
        cwd         => "/srv/maverick/software/opencv/build",
        creates     => "/srv/maverick/software/opencv/build/Makefile",
    } ->
    exec { "opencv-build":
        user        => "mav",
        timeout     => 0,
        command     => "/usr/bin/make -j${::processorcount} >/var/tmp/opencv.build.out 2>&1",
        cwd         => "/srv/maverick/software/opencv/build",
        creates     => "/srv/maverick/software/opencv/build/opencv",
    }
    
}