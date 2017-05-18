# Vision Module
Vision is an important (and fun) part of UAVs.  Maverick contains good basic support for the standard elements of Computer Vision and Video functionality.

- **Gstreamer**: Industry standard software for capturing, transcoding and transmitting video
- **OpenCV**: Industry standard software for Computer Vision
- **Aruco**: Fiducial Tag library, for recognising tags/markers in video
- **Orb_Slam2**: Fast library for performing monocular and stereo SLAM
- **Visiond**: A dynamic service that detects camera and encoding hardware, and automatically generates a gstreamer pipeline to transmit the video over the network - eg. wifi.  Very useful for FPV, it will in the future be useful for transmitting CV and other video
- **Vision_landing**: Software that uses tags/markers (Aruco) as landing patterns, and controls Precision Landing in ArduCopter
- **Wifibroadcast**: Innovative software that uses monitor/inject mode of compatible wifi adapters to provide connection-less wifi video with graceful degradation, similar to traditional analogue FPV.

One key advantage of Maverick is that wherever possible it provides the same versions of software across all platforms.  This is very useful for porting code and functionality across platforms, as the underlying components often vary widely in their APIs/features.  As of Maverick 1.0, the component versions are:  
- Gstreamer: **1.10.4**
- OpenCV: **3.2**
- ROS: **Kinetic**  

There are a few exceptions, eg. Raspberry gstreamer has extensive modifications made to support the Raspberry VPU that handles the hardware video encoding so this platform is stuck with a very old version of gstreamer (1.4).

### Gstreamer
Apart from the Raspberry platform as mentioned above, Gstreamer is a set version compiled from source on all platforms.  It includes certain optimizations and extra modules where appropriate for the platform - relevant hardware encoding modules for Odroid and Joule platforms.

The standard source-compiled gstreamer is installed into ~/software/gstreamer, and the necessary supporting environment is automatically set.  All other standard source-compiled Maverick software components such as OpenCV, ROS, Aruco, visiond, vision_landing etc all use this version of gstreamer.

It is possible to set the version of gstreamer in a localconf parameter (note this requires clearing the gstreamer install flag and waiting for a recompile, which can take several hours):  
`"maverick_vision::gstreamer::gstreamer_version": "1.12.0"`  
It is possible to install gstreamer from system packages by setting localconf parameter:  
`"maverick_vision::gstreamer::gstreamer_installtype": "native"`  

### OpenCV
Like gstreamer, OpenCV is compiled from source for a consistent version across all platforms.  Most current OS distributions provide the elderly OpenCV2.  OpenCV3 is a major release with many improvements and new features, Maverick provides the first stable release - 3.2.0.  This version can be changed by setting localconf parameter:  
`"maverick_vision::opencv::opencv_version": "3.x.x"`  
Note that OpenCV takes a **long** time to compile on slower platforms (6 hours+ on a Raspberry).  Opencv-contrib is included as standard, this can be disabled during compile by setting localconf parameter:  
`"maverick_vision::opencv::contrib": false`  
There are two compile options which rarely need to be changed:  
 - `"maverick_vision::opencv::release": "Release"` can be changed to 'Debug' to produced debug libraries/binaries
 - `"maverick_vision::opencv::precompile_headers": false'` control how the compile process uses precompile to speed up compilation.  However, it uses vast amounts of disk space so is turned off by default in Maverick

OpenCV is installed into ~/software/opencv and the necessary supporting environment is automatically set.

### Visiond
Visiond is a Maverick-specific (python-based) daemon that attempts to automate the process of capturing, transcoding and transmitting video over the network.  It detects the attached camera and encoding hardware and constructs a gstreamer pipeline based on the detected hardware details.  There is a dynamic config file in ~/data/config/vision/maverick-visiond.conf that allows easy configuration of the device, video format, resolution, framerate and network output.  To activate the config changes, restart the service:  
`maverick restart visiond`
This service is started by default.

### Vision_landing
vision_landing combines the Aruco, Gstreamer, OpenCV and optionally RealSense components to create a system that analyses video in realtime for landing markers (Aruco/April fiducial markers, or tags) and uses these markers to estimate the position and distance of the landing marker compared to the UAV and passes this data to the flight controller.  The flight controller applies corrections according to the attitude of the UAV to work out what needs to be done to land on the marker.

It is a work in progress (as is the support for vision based landing in ArduPilot) - the main project page is https://github.com/fnoop/vision_landing.

Like visiond, it has a dynamic config in ~/data/config/vision/vision_landing.conf, and to activate any config changes, restart the service:  
`maverick restart vision_landing`
This servce is not started by default, as it contends for camera usage with visiond.  To use it, first turn off visiond (`maverick stop visiond`).

### Orb_slam2
Orb_slam2 is not installed by default, as it takes a lot of resource and is really an experimental/academic project - of limited real-world use.  To enable it, set localconf parameter:  
`"maverick_vision::orb_slam2": true`  
