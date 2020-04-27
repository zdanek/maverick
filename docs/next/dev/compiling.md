# Compiling
Maverick deliberately isolates provided third party and it's own software components in a single location: */srv/maverick/software*.  The purpose of this is to:  
  - Provide a single location for software
  - Provide the latest, consistent software components and versions across different platforms
  - Provide a known, reliable level and location of software, isolated from any OS packages which tend to lag behind in versions

Maverick includes numerous third party software components, including for example the popular OpenCV and Gstreamer projects which are often used to compile user projects.  Because they live in an unexpected location (eg. */srv/maverick/software/opencv* and */srv/maverick/software/gstreamer*), Maverick also provides environment settings and configuration to facilitate easy use.

## Environment Settings and Configuration

### Profile.d Environment Variables
Maverick uses the *profile.d* facility to add environmental variables that are commonly used by compilers and associated software to find components that can be used to compile and link.  Various Maverick modules and manifests place files into */etc/profile.d*, and these files are picked up and executed as part of interactive shell initialization.  For example, the 'maverick_vision::opencv' manifest places several files into /etc/profile.d:
 - *40-maverick-opencv-cmake.sh*: Adds OpenCV software path (*/srv/maverick/software/opencv) to CMAKE_PREFIX_PATH, which is used by cmake to find the OpenCV component.
 - *40-maverick-opencv-ldlibrarypath.sh*: Adds OpenCV library path (*/srv/maverick/software/opencv/lib*) to LD_LIBRARY_PATH, which is used by various compilers and linking mechanisms to find the OpenCV shared libraries.
 - *40-maverick-opencv-path.sh*: Adds OpenCV bin directory to the front of the user PATH (highest priority so they override any conflicting system software).
 - *40-maverick-opencv-pkgconfig.sh*: Adds OpenCV pkg-config path (*/srv/maverick/software/opencv/lib/pkgconfig*) to PKG_CONFIG_PATH, which compilers or other software components use with 'pkg-config' tool to determine component paths.
 - *40-maverick-opencv-pythonpath.sh*: Adds OpenCV python module path (*/srv/maverick/software/opencv/lib/python3.7/site-packages*) to PYTHONPATH so Python can find the module.

All these environment settings provide a wide variety of ways for the user, compilers, linkers and other software to find OpenCV (and other software copmonents that Maverick provides), and usually do so with a higher priority than any system software as we want our custom software components used in preference.

### Ldconfig configuration
In addition to environment variables, Maverick also sets a config file into */etc/ld.so.conf.d* that adds the component library path to the system linker.  For example the 'maverick_vision::opencv' manifest places */etc/ld.so.conf.d/maverick-opencv.conf*, which adds the custom OpenCV shared libraries to the system linker.

As with the environment variables, Maverick also setups ldconfig for most custom software components.
!> ldconfig always sets standard system paths (eg. /usr/lib) as a higher priority than custom paths, so existing system components may interefere with our custom components even when defined in this way.  For this reason, the profile.d environment variables tend to be more reliable than ldconfig.

## Compiling User Software

### Compiling outside of Maverick manifests
Compiling software that takes advantage of the Maverick custom software components is relatively straight forward, given the above pre-set variables and config.  As an example, we use GoodRobots [vision_landing](https://github.com/goodrobots/vision_landing) software.  vision_landing requires OpenCV and Aruco to compile - here is how we download and compile the software in the Maverick environment:  
```bash
cd ~/code
git clone https://github.com/goodrobots/vision_landing
cd vision_landing/src
cmake .
make
```

That's it!  That should compile cleanly, using our custom Aruco from ~/software/aruco and our custom OpenCV from ~/software/opencv.  You can test this by doing:
```bash
ldd track_targets |egrep 'aruco|opencv'
libaruco.so.3.0 => /srv/maverick/software/aruco/lib/libaruco.so.3.0 (0x00007f026d0f9000)
libopencv_highgui.so.4.2 => /srv/maverick/software/opencv/lib/libopencv_highgui.so.4.2 (0x00007f026ccc2000)
libopencv_videoio.so.4.2 => /srv/maverick/software/opencv/lib/libopencv_videoio.so.4.2 (0x00007f026ca44000)
libopencv_calib3d.so.4.2 => /srv/maverick/software/opencv/lib/libopencv_calib3d.so.4.2 (0x00007f026c69b000)
libopencv_photo.so.4.2 => /srv/maverick/software/opencv/lib/libopencv_photo.so.4.2 (0x00007f026be95000)
libopencv_imgproc.so.4.2 => /srv/maverick/software/opencv/lib/libopencv_imgproc.so.4.2 (0x00007f026a56d000)
libopencv_core.so.4.2 => /srv/maverick/software/opencv/lib/libopencv_core.so.4.2 (0x00007f026921a000)
libopencv_ml.so.4.2 => /srv/maverick/software/opencv/lib/libopencv_ml.so.4.2 (0x00007f0268223000)
libopencv_imgcodecs.so.4.2 => /srv/maverick/software/opencv/lib/libopencv_imgcodecs.so.4.2 (0x00007f02678d7000)
libopencv_features2d.so.4.2 => /srv/maverick/software/opencv/lib/libopencv_features2d.so.4.2 (0x00007f0262bf3000)
libopencv_flann.so.4.2 => /srv/maverick/software/opencv/lib/libopencv_flann.so.4.2 (0x00007f0262995000)
```

### Compiling in Maverick manifests
Paradoxically, compiling software in Maverick manifests is actually more difficult, because Puppet (the underlying Configuration Management system that performs the `configure` action) deliberately suppresses all environment variables other than core system values for safety.  So we have to add these back into the manifests.  Again we use vision_landing as an example - here is a manifest fragment to compile vision_landing:  
```puppet
# Install vision_landing
oncevcsrepo { "git-vision_landing":
    gitsource   => $vision_landing_source,
    dest        => "/srv/maverick/software/vision_landing",
    revision    => $vision_landing_revision,
    depth       => 0,
} ->
# Compile vision_landing
exec { "vision_landing-compile":
    user        => mav,
    environment => ["LD_LIBRARY_PATH=/srv/maverick/software/opencv/lib:/srv/maverick/software/aruco/lib", "CMAKE_PREFIX_PATH=/srv/maverick/software/opencv:/srv/maverick/software/aruco:/srv/maverick/software/librealsense", "CMAKE_INSTALL_RPATH=/srv/maverick/software/aruco/lib:/srv/maverick/software/opencv/lib:/srv/maverick/software/librealsense"],
    cwd         => "/srv/maverick/software/vision_landing/src",
    command     => "/usr/bin/cmake -Daruco_DIR=/srv/maverick/software/aruco -DOpenCV_DIR=/srv/maverick/software/opencv -DCMAKE_INSTALL_RPATH=/srv/maverick/software/aruco/lib:/srv/maverick/software/opencv/lib -DCMAKE_MODULE_PATH=/srv/maverick/software/opencv:/srv/maverick/software/aruco . && make && make install",
    creates     => "/srv/maverick/software/vision_landing/track_targets",
    require     => [ Class["maverick_vision::opencv"], Class["maverick_vision::aruco"] ],
}
```

Here we can see that we set environment variables that point to the various custom software components, and also set cmake directives.  In addition, it sets the RPATH which is a runtime hinter so the installed software can still find the custom shared library components.
This will be improved in the future so most of this will not be necessary - the environment variables will be set and honoured even in Maverick manifests.