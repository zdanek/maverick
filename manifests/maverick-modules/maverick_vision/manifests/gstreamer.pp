class maverick_vision::gstreamer (
    $gstreamer_installtype = "source",
    $gstreamer_version = "1.12.1",
    $libx264 = "installed",
) {
    # Install gstreamer from binary packages.  If raspberry, override installtype must install binary.
    if $gstreamer_installtype == "native" or $raspberry_present == "yes" {
        ensure_packages(["libgstreamer1.0-0", "libgstreamer-plugins-base1.0-dev", "libgstreamer1.0-dev", "gstreamer1.0-alsa", "gstreamer1.0-plugins-base", "gstreamer1.0-plugins-bad", "gstreamer1.0-plugins-ugly", "gstreamer1.0-tools", "python-gst-1.0", "gir1.2-gstreamer-1.0", "gir1.2-gst-plugins-base-1.0", "gir1.2-clutter-gst-2.0", "python-gi"])
        if ($raspberry_present == "yes") {
    		ensure_packages(["gstreamer1.0-omx"])
            # Even if raspberry gstreamer is binary install, it doesn't include rtsp so install from source
            if ! ("install_flag_gst-rtsp-server" in $installflags) {
                file { "/srv/maverick/var/build/gstreamer":
                    ensure      => directory,
                    owner       => "mav",
                    group       => "mav",
                    mode        => "755",
                } ->
                oncevcsrepo { "git-gstreamer_gst_rtsp_server":
                    gitsource   => "https://github.com/GStreamer/gst-rtsp-server.git",
                    dest        => "/srv/maverick/var/build/gstreamer/gst-rtsp-server",
                    revision    => "1.4.4",
                } ->
                exec { "gstreamer_gst_rtsp_server":
                    timeout     => 0,
                    environment => ["PKG_CONFIG_PATH=/usr/lib/arm-linux-gnueabihf/pkgconfig"],
                    command     => "/srv/maverick/var/build/gstreamer/gst-rtsp-server/autogen.sh --libdir=/usr/lib/arm-linux-gnueabihf --disable-gtk-doc --prefix=/usr && /usr/bin/make -j${::processorcount} && /usr/bin/make install >/srv/maverick/var/log/build/gstreamer_gst_rtsp_server.build.out 2>&1",
                    cwd         => "/srv/maverick/var/build/gstreamer/gst-rtsp-server",
                    creates     => "/usr/lib/arm-linux-gnueabihf/libgstrtspserver-1.0.so",
                    require     => [ Oncevcsrepo["git-gstreamer_gst_rtsp_server"] ]
                } ->
                file { "/srv/maverick/var/build/.install_flag_gst-rtsp-server":
                    ensure      => present,
                    owner       => "mav",
                    group       => "mav",
                    mode        => "644",
                }
            }
        }
        # If odroid and MFC v4l device present, compile and install the custom gstreamer codecs
        if $odroid_present == "yes" and $camera_odroidmfc == "yes" and 1 == 2 {
            ensure_packages(["libgudev-1.0-dev", "dh-autoreconf", "automake", "autoconf", "libtool", "autopoint", "cdbs", "gtk-doc-tools", "dpkg-dev"])
            ensure_packages(["libshout3-dev", "libaa1-dev", "libflac-dev", "libsoup2.4-dev", "libraw1394-dev", "libiec61883-dev", "libavc1394-dev", "liborc-0.4-dev", "libcaca-dev", "libdv4-dev", "libxv-dev", "libgtk-3-dev", "libtag1-dev", "libwavpack-dev", "libpulse-dev", "libjack-jackd2-dev", "libvpx-dev"])
            exec { "odroidmfc-package":
                command     => "/usr/bin/dpkg-buildpackage -us -uc -b -j4",
                cwd         => "/srv/maverick/var/build/gstreamer_odroidmfc/gst-plugins-good",
                creates     => "/srv/maverick/var/build/gstreamer_odroidmfc/gstreamer1.0-plugins-good_1.8.2-1ubuntu3_armhf.deb",
                timeout     => 0,
                require     => Oncevcsrepo["git-gstreamer_odroidmfc"],
            } ->
            package { "libgstreamer-plugins-good1.0-0":
                provider    => dpkg,
                ensure      => latest,
                source      => "/srv/maverick/var/build/gstreamer_odroidmfc/libgstreamer-plugins-good1.0-0_1.8.2-1ubuntu3_armhf.deb",
            } ->
            package { "gstreamer1.0-plugins-good":
                provider    => dpkg,
                ensure      => latest,
                source      => "/srv/maverick/var/build/gstreamer_odroidmfc/gstreamer1.0-plugins-good_1.8.2-1ubuntu3_armhf.deb",
            }
        } else {
            package { "gstreamer1.0-plugins-good":
                ensure      => present,
            }
        }
        
        # Create a blank exec as a resource for other manifests to use as a dependency
        # exec { "gstreamer-installed": }

	} elsif $gstreamer_installtype == "source" {
        # Work out which gst-plugins-good we want, if we're an odroid with active MFC device use patched tree for hardware codec
        if $odroid_present == "yes" and $camera_odroidmfc == "yes" {
            $gst_plugins_good_src = "https://github.com/fnoop/gst-plugins-good.git"
            $gst_plugins_good_revision = "1.10.2-patched"
            ensure_packages(["libgudev-1.0-dev", "dh-autoreconf", "automake", "autoconf", "libtool", "autopoint", "cdbs", "gtk-doc-tools", "dpkg-dev"])
            ensure_packages(["libshout3-dev", "libaa1-dev", "libflac-dev", "libsoup2.4-dev", "libraw1394-dev", "libiec61883-dev", "libavc1394-dev", "liborc-0.4-dev", "libcaca-dev", "libdv4-dev", "libxv-dev", "libgtk-3-dev", "libtag1-dev", "libwavpack-dev", "libpulse-dev", "libjack-jackd2-dev", "libvpx-dev"])
            ensure_packages(["mesa-utils"])
        } else {
            $gst_plugins_good_src = "https://github.com/GStreamer/gst-plugins-good.git"
            $gst_plugins_good_revision = $gstreamer_version
        }
        
        # Work out libpython path
        if $architecture == "armv7l" or $architecture == "armv6l" {
            $libpython_path = "/usr/lib/arm-linux-gnueabihf"
        } elsif $architecture == "amd64" {
            $libpython_path = "/usr/lib/x86_64-linux-gnu"
        }

        # Compile and install gstreamer from source, unless the install flag ~/var/build/.install_flag_gstreamer is already set
        if ! ("install_flag_gstreamer" in $installflags) {

            # Install necessary dependencies and compile
            ensure_packages(["libglib2.0-dev", "autogen", "autoconf", "autopoint", "libtool-bin", "bison", "flex", "gtk-doc-tools", "python-gobject", "python-gobject-dev", "gobject-introspection", "libgirepository1.0-dev", "liborc-0.4-dev", "python-gi", "nasm", "libxext-dev"])
            package { ["libx264-dev"]:
                ensure      => $libx264,
            } ->
            #package { ["gstreamer*", "libgstreamer*", "python-gst-1.0"]:
            #    ensure      => absent,
            #} ->
    
            file { "/srv/maverick/var/build/gstreamer":
                ensure      => directory,
                owner       => "mav",
                group       => "mav",
                mode        => "755",
            } ->

            oncevcsrepo { "git-gstreamer_core":
                gitsource   => "https://github.com/GStreamer/gstreamer.git",
                dest        => "/srv/maverick/var/build/gstreamer/core",
                revision    => $gstreamer_version,
            } ->
            oncevcsrepo { "git-gstreamer_plugins_base":
                gitsource   => "https://github.com/GStreamer/gst-plugins-base.git",
                dest        => "/srv/maverick/var/build/gstreamer/gst-plugins-base",
                revision    => $gstreamer_version,
            } ->
            oncevcsrepo { "git-gstreamer_plugins_good":
                gitsource   => $gst_plugins_good_src,
                dest        => "/srv/maverick/var/build/gstreamer/gst-plugins-good",
                revision    => $gst_plugins_good_revision,
            } ->
            oncevcsrepo { "git-gstreamer_plugins_bad":
                gitsource   => "https://github.com/GStreamer/gst-plugins-bad.git",
                dest        => "/srv/maverick/var/build/gstreamer/gst-plugins-bad",
                revision    => $gstreamer_version,
            } ->
            oncevcsrepo { "git-gstreamer_plugins_ugly":
                gitsource   => "https://github.com/GStreamer/gst-plugins-ugly.git",
                dest        => "/srv/maverick/var/build/gstreamer/gst-plugins-ugly",
                revision    => $gstreamer_version,
            } ->
            oncevcsrepo { "git-gstreamer_libav":
                gitsource   => "https://github.com/GStreamer/gst-libav.git",
                dest        => "/srv/maverick/var/build/gstreamer/gst-libav",
                revision    => $gstreamer_version,
            } ->
            oncevcsrepo { "git-gstreamer_gst_python":
                gitsource   => "https://github.com/GStreamer/gst-python.git",
                dest        => "/srv/maverick/var/build/gstreamer/gst-python",
                revision    => $gstreamer_version,
            } ->
            oncevcsrepo { "git-gstreamer_gst_rtsp_server":
                gitsource   => "https://github.com/GStreamer/gst-rtsp-server.git",
                dest        => "/srv/maverick/var/build/gstreamer/gst-rtsp-server",
                revision    => $gstreamer_version,
            } ->
            
            exec { "gstreamer_core-build":
                user        => "mav",
                timeout     => 0,
                environment => ["PKG_CONFIG_PATH=/srv/maverick/software/gstreamer/lib/pkgconfig", "LDFLAGS=-Wl,-rpath,/srv/maverick/software/gstreamer/lib"],
                command     => "/srv/maverick/var/build/gstreamer/core/autogen.sh --disable-gtk-doc --disable-docbook --prefix=/srv/maverick/software/gstreamer && /usr/bin/make -j${::processorcount} && /usr/bin/make install >/srv/maverick/var/log/build/gstreamer_core.build.out 2>&1",
                cwd         => "/srv/maverick/var/build/gstreamer/core",
                creates     => "/srv/maverick/software/gstreamer/bin/gst-launch-1.0",
                require     => [ Package["libglib2.0-dev", "bison", "flex"], Oncevcsrepo["git-gstreamer_core"], Package["libgirepository1.0-dev"] ] # ensure we have all the dependencies satisfied
            }->
            exec { "gstreamer_gst_plugins_base":
                user        => "mav",
                timeout     => 0,
                environment => ["PKG_CONFIG_PATH=/srv/maverick/software/gstreamer/lib/pkgconfig", "LDFLAGS=-Wl,-rpath,/srv/maverick/software/gstreamer/lib"],
                command     => "/srv/maverick/var/build/gstreamer/gst-plugins-base/autogen.sh --disable-gtk-doc --with-pkg-config-path=/srv/maverick/software/gstreamer/lib/pkgconfig --prefix=/srv/maverick/software/gstreamer && /usr/bin/make -j${::processorcount} && /usr/bin/make install >/srv/maverick/var/log/build/gstreamer_plugins_base.build.out 2>&1",
                cwd         => "/srv/maverick/var/build/gstreamer/gst-plugins-base",
                creates     => "/srv/maverick/software/gstreamer/bin/gst-play-1.0",
                require     => [ Oncevcsrepo["git-gstreamer_plugins_base"], Package["libgirepository1.0-dev"], Exec["gstreamer_core-build"] ]
            } ->
            exec { "gstreamer_gst_plugins_good":
                user        => "mav",
                timeout     => 0,
                environment => ["PKG_CONFIG_PATH=/srv/maverick/software/gstreamer/lib/pkgconfig", "LDFLAGS=-Wl,-rpath,/srv/maverick/software/gstreamer/lib"],
                command     => "/srv/maverick/var/build/gstreamer/gst-plugins-good/autogen.sh --disable-gtk-doc --with-pkg-config-path=/srv/maverick/software/gstreamer/lib/pkgconfig --enable-v4l2-probe --with-libv4l2 --prefix=/srv/maverick/software/gstreamer && /usr/bin/make -j${::processorcount} && /usr/bin/make install >/srv/maverick/var/log/build/gstreamer_plugins_good.build.out 2>&1",
                cwd         => "/srv/maverick/var/build/gstreamer/gst-plugins-good",
                creates     => "/srv/maverick/software/gstreamer/lib/gstreamer-1.0/libgstjpeg.so",
                require     => [ Oncevcsrepo["git-gstreamer_plugins_good"], Exec["gstreamer_gst_plugins_base"] ]
            }
            if $raspberry_present == "yes" {
                exec { "gstreamer_gst_plugins_bad":
                    user        => "mav",
                    timeout     => 0,
                    environment => [
                        "CFLAGS=-I/opt/vc/include -I/opt/vc/include/interface/vcos/pthreads -I/opt/vc/include/interface/vmcs_host/linux",
                        "CPPFLAGS=-I/opt/vc/include -I/opt/vc/include/interface/vcos/pthreads -I/opt/vc/include/interface/vmcs_host/linux",
                        "LDFLAGS=-L/opt/vc/lib -Wl,-rpath,/srv/maverick/software/gstreamer/lib",
                        "PKG_CONFIG_PATH=/srv/maverick/software/gstreamer/lib/pkgconfig", 
                        "EGL_CFLAGS=/opt/vc/include",
                        "EGL_LIBS=/opt/vc/lib",
                        "GLES2_CFLAGS=/opt/vc/include",
                        "GLES2_LIBS=/opt/vc/lib"
                    ],
                    # command     => "/srv/maverick/var/build/gstreamer/gst-plugins-bad/autogen.sh --disable-gtk-doc --disable-examples --disable-x11 --disable-qt --enable-egl -enable-opengl --with-pkg-config-path=/srv/maverick/software/gstreamer/lib/pkgconfig --prefix=/srv/maverick/software/gstreamer && /usr/bin/make -j${::processorcount} CFLAGS+='-Wno-error -Wno-redundant-decls -I/opt/vc/include -I/opt/vc/include/interface/vcos/pthreads -I/opt/vc/include/interface/vmcs_host/linux' CPPFLAGS+='-Wno-error -Wno-redundant-decls -I/opt/vc/include -I/opt/vc/include/interface/vcos/pthreads -I/opt/vc/include/interface/vmcs_host/linux'  CXXFLAGS+='-Wno-redundant-decls' LDFLAGS+='-L/opt/vc/lib' && /usr/bin/make install >/srv/maverick/var/log/build/gstreamer_plugins_bad.build.out 2>&1",
                    # command     => "/srv/maverick/var/build/gstreamer/gst-plugins-bad/autogen.sh --disable-gtk-doc --disable-examples --disable-x11 --disable-qt --enable-egl --enable-gles2 --with-gles2-module-name=/opt/vc/lib/libGLESv2.so --with-egl-module-name=/opt/vc/lib/libEGL.so --with-pkg-config-path=/srv/maverick/software/gstreamer/lib/pkgconfig --prefix=/srv/maverick/software/gstreamer && make && make install >/srv/maverick/var/log/build/gstreamer_plugins_bad.build.out 2>&1",
                    command     => "/srv/maverick/var/build/gstreamer/gst-plugins-bad/autogen.sh --disable-gtk-doc --enable-egl --enable-gles2 --disable-opengl --with-pkg-config-path=/srv/maverick/software/gstreamer/lib/pkgconfig --prefix=/srv/maverick/software/gstreamer && make && make install >/srv/maverick/var/log/build/gstreamer_plugins_bad.build.out 2>&1",
                    cwd         => "/srv/maverick/var/build/gstreamer/gst-plugins-bad",
                    creates     => "/srv/maverick/software/gstreamer/lib/libgstgl-1.0.so",
                    require     => [ Oncevcsrepo["git-gstreamer_plugins_bad"], Exec["gstreamer_gst_plugins_base"] ]
                }
            } else {
                exec { "gstreamer_gst_plugins_bad":
                    user        => "mav",
                    timeout     => 0,
                    environment => ["PKG_CONFIG_PATH=/srv/maverick/software/gstreamer/lib/pkgconfig", "LDFLAGS=-Wl,-rpath,/srv/maverick/software/gstreamer/lib"],
                    command     => "/srv/maverick/var/build/gstreamer/gst-plugins-bad/autogen.sh --disable-gtk-doc --disable-qt --with-pkg-config-path=/srv/maverick/software/gstreamer/lib/pkgconfig --prefix=/srv/maverick/software/gstreamer && /usr/bin/make -j${::processorcount} && /usr/bin/make install >/srv/maverick/var/log/build/gstreamer_plugins_bad.build.out 2>&1",
                    cwd         => "/srv/maverick/var/build/gstreamer/gst-plugins-bad",
                    creates     => "/srv/maverick/software/gstreamer/lib/libgstbadbase-1.0.so",
                    require     => [ Oncevcsrepo["git-gstreamer_plugins_bad"], Exec["gstreamer_gst_plugins_base"] ]
                }
            }
            exec { "gstreamer_gst_plugins_ugly":
                user        => "mav",
                timeout     => 0,
                environment => ["PKG_CONFIG_PATH=/srv/maverick/software/gstreamer/lib/pkgconfig", "LDFLAGS=-Wl,-rpath,/srv/maverick/software/gstreamer/lib"],
                command     => "/srv/maverick/var/build/gstreamer/gst-plugins-ugly/autogen.sh --disable-gtk-doc --with-pkg-config-path=/srv/maverick/software/gstreamer/lib/pkgconfig --prefix=/srv/maverick/software/gstreamer && /usr/bin/make -j${::processorcount} && /usr/bin/make install >/srv/maverick/var/log/build/gstreamer_plugins_ugly.build.out 2>&1",
                cwd         => "/srv/maverick/var/build/gstreamer/gst-plugins-ugly",
                creates     => "/srv/maverick/software/gstreamer/share/locale/en_GB/LC_MESSAGES/gst-plugins-ugly-1.0.mo",
                require     => [ Package["libx264-dev"], Oncevcsrepo["git-gstreamer_plugins_ugly"], Exec["gstreamer_gst_plugins_base"] ]
            } ->
            exec { "gstreamer_gst_libav":
                user        => "mav",
                timeout     => 0,
                environment => ["PKG_CONFIG_PATH=/srv/maverick/software/gstreamer/lib/pkgconfig", "LDFLAGS=-Wl,-rpath,/srv/maverick/software/gstreamer/lib", "AS=gcc"],
                command     => "/srv/maverick/var/build/gstreamer/gst-libav/autogen.sh --disable-gtk-doc --with-pkg-config-path=/srv/maverick/software/gstreamer/lib/pkgconfig --prefix=/srv/maverick/software/gstreamer && /usr/bin/make -j${::processorcount} && /usr/bin/make install >/srv/maverick/var/log/build/gstreamer_libav.build.out 2>&1",
                cwd         => "/srv/maverick/var/build/gstreamer/gst-libav",
                creates     => "/srv/maverick/software/gstreamer/lib/gstreamer-1.0/libgstlibav.so",
                require     => [ Package["libx264-dev"], Oncevcsrepo["git-gstreamer_libav"], Exec["gstreamer_gst_plugins_base"] ]
            } ->
            exec { "gstreamer_gst_python":
                user        => "mav",
                timeout     => 0,
                environment => ["PKG_CONFIG_PATH=/srv/maverick/software/gstreamer/lib/pkgconfig", "LDFLAGS=-Wl,-rpath,/srv/maverick/software/gstreamer/lib"],
                command     => "/srv/maverick/var/build/gstreamer/gst-python/autogen.sh --with-libpython-dir=${libpython_path} --prefix=/srv/maverick/software/gstreamer && /usr/bin/make -j${::processorcount} && /usr/bin/make install >/srv/maverick/var/log/build/gstreamer_gst_python.build.out 2>&1",
                cwd         => "/srv/maverick/var/build/gstreamer/gst-python",
                creates     => "/srv/maverick/software/gstreamer/lib/gstreamer-1.0/libgstpythonplugin.so",
                require     => [ Package["python-gobject-dev"], Oncevcsrepo["git-gstreamer_gst_python"], Exec["gstreamer_gst_plugins_base"] ]
            } ->
            exec { "gstreamer_gst_rtsp_server":
                user        => "mav",
                timeout     => 0,
                environment => ["PKG_CONFIG_PATH=/srv/maverick/software/gstreamer/lib/pkgconfig", "LDFLAGS=-Wl,-rpath,/srv/maverick/software/gstreamer/lib"],
                command     => "/srv/maverick/var/build/gstreamer/gst-rtsp-server/autogen.sh --disable-gtk-doc --with-pkg-config-path=/srv/maverick/software/gstreamer/lib/pkgconfig --prefix=/srv/maverick/software/gstreamer && /usr/bin/make -j${::processorcount} && /usr/bin/make install >/srv/maverick/var/log/build/gstreamer_gst_rtsp_server.build.out 2>&1",
                cwd         => "/srv/maverick/var/build/gstreamer/gst-rtsp-server",
                creates     => "/srv/maverick/software/gstreamer/lib/libgstrtspserver-1.0.so",
                require     => [ Oncevcsrepo["git-gstreamer_gst_rtsp_server"], Exec["gstreamer_gst_plugins_base"] ]
            } ->
            file { "/srv/maverick/var/build/.install_flag_gstreamer":
                ensure      => present,
            }
    
            # Install vaapi for joule platform
            if $joule_present == "yes" {
                ensure_packages(["libdrm-dev", "libudev-dev", "libxrandr-dev"])
                oncevcsrepo { "git-gstreamer_vaapi":
                    gitsource   => "https://github.com/GStreamer/gstreamer-vaapi.git",
                    dest        => "/srv/maverick/var/build/gstreamer/gstreamer-vaapi",
                    revision    => $gstreamer_version,
                } ->
                exec { "gstreamer_vaapi":
                    user        => "mav",
                    timeout     => 0,
                    environment => ["PKG_CONFIG_PATH=/srv/maverick/software/gstreamer/lib/pkgconfig", "LDFLAGS=-Wl,-rpath,/srv/maverick/software/gstreamer/lib"],
                    command     => "/srv/maverick/var/build/gstreamer/gstreamer-vaapi/autogen.sh --disable-gtk-doc --with-pkg-config-path=/srv/maverick/software/gstreamer/lib/pkgconfig --prefix=/srv/maverick/software/gstreamer && /usr/bin/make -j${::processorcount} && /usr/bin/make install >/srv/maverick/var/log/build/gstreamer_vaapi.build.out 2>&1",
                    cwd         => "/srv/maverick/var/build/gstreamer/gstreamer-vaapi",
                    creates     => "/srv/maverick/software/gstreamer/lib/gstreamer-1.0/libgstvaapi.so",
                    require     => [ Exec["gstreamer_gst_plugins_base"], Package["libva1"], Package["libdrm-dev"] ]
                }
            }
            
            # Recent gstreamer OMX broken on raspberry, must install raspbian binary packages
            # See https://github.com/fnoop/maverick/issues/242
            if ($raspberry_present == "yes") {
                oncevcsrepo { "git-gstreamer_omx":
                    gitsource   => "https://github.com/fnoop/gst-omx-rpi.git",
                    dest        => "/srv/maverick/var/build/gstreamer/gst-omx",
                } ->
                exec { "gstreamer_gst_omx":
                    user        => "mav",
                    timeout     => 0,
                    environment => [
                        "CFLAGS=-DOMX_SKIP64BIT -I/opt/vc/include -I/opt/vc/include/IL -I/opt/vc/include/interface/vcos/pthreads -I/opt/vc/include/interface/vmcs_host/linux",
                        "CPPFLAGS=-I/opt/vc/include -I/opt/vc/include/IL -I/opt/vc/include/interface/vcos/pthreads -I/opt/vc/include/interface/vmcs_host/linux",
                        "LDFLAGS=-L/opt/vc/lib -Wl,-rpath,/srv/maverick/software/gstreamer/lib",
                        "PKG_CONFIG_PATH=/srv/maverick/software/gstreamer/lib/pkgconfig" 
                    ],
                    command     => "/srv/maverick/var/build/gstreamer/gst-omx/autogen.sh --with-omx-target=rpi --disable-gtk-doc --disable-examples --prefix=/srv/maverick/software/gstreamer; /usr/bin/make -j${::processorcount}; /usr/bin/make install >/srv/maverick/var/log/build/gstreamer_omx.build.out 2>&1",
                    cwd         => "/srv/maverick/var/build/gstreamer/gst-omx",
                    creates     => "/srv/maverick/software/gstreamer/lib/gstreamer-1.0/libgstomx.so",
                    require     => [ Oncevcsrepo["git-gstreamer_omx"], Exec["gstreamer_gst_plugins_base"] ]
                } ->
                file { "/etc/xdg":
                    ensure      => directory,
                } ->
                exec { "cp-xdg-conf":
                    command     => "/bin/cp /srv/maverick/var/build/gstreamer/gst-omx/config/rpi/gstomx.conf /etc/xdg",
                    unless      => "/bin/ls /etc/xdg/gstomx.conf",
                }
            }
    	} else {
    	    # If we don't build gstreamer, we still need something for other manifest dependencies
    	    file { "/srv/maverick/var/build/.install_flag_gstreamer":
                ensure      => present,
            }
    	}
	
        # Export local typelib for gobject introspection
        file { "/etc/profile.d/50-gi-typelibs.sh":
            ensure      => present,
            mode        => "644",
            owner       => "root",
            group       => "root",
            content     => "export GI_TYPELIB_PATH=/srv/maverick/software/gstreamer/lib/girepository-1.0:/usr/lib/girepository-1.0",
        }

        # Set profile scripts for custom gstreamer location
        file { "/etc/profile.d/50-maverick-gstreamer-path.sh":
            mode        => "644",
            content     => "export PATH=/srv/maverick/software/gstreamer/bin:\$PATH",
        }
        file { "/etc/profile.d/50-maverick-gstreamer-pkgconfig.sh":
            mode        => "644",
            owner       => "root",
            group       => "root",
            content     => "export PKG_CONFIG_PATH=/srv/maverick/software/gstreamer/lib/pkgconfig:\$PKG_CONFIG_PATH",
        }
        file { "/etc/profile.d/50-maverick-gstreamer-plugins.sh":
            mode        => "644",
            owner       => "root",
            group       => "root",
            content     => "export GST_PLUGIN_PATH=/srv/maverick/software/gstreamer/lib/gstreamer-1.0; export GST_PLUGIN_SYSTEM_PATH=\$GST_PLUGIN_PATH",
        }
        file { "/etc/profile.d/50-maverick-gstreamer-pythonpath.sh":
            mode        => "644",
            owner       => "root",
            group       => "root",
            content     => "export PYTHONPATH=/srv/maverick/software/gstreamer/lib/python2.7/site-packages:\$PYTHONPATH",
        }
        file { "/etc/profile.d/50-maverick-gstreamer-ldlibrarypath.sh":
            ensure      => absent,
        }
        file { "/etc/ld.so.conf.d/maverick-gstreamer.conf":
            mode        => "644",
            owner       => "root",
            group       => "root",
            content     => "/srv/maverick/software/gstreamer/lib",
            notify      => Exec["maverick-ldconfig"],
        } ->
        file { "/etc/profile.d/50-maverick-gstreamer-cmake.sh":
            mode        => "644",
            owner       => "root",
            group       => "root",
            content     => "export CMAKE_PREFIX_PATH=/srv/maverick/software/gstreamer:\$CMAKE_PREFIX_PATH",
        }

        # Create a blank exec as a resource for other manifests to use as a dependency
        # exec { "gstreamer-installed": }
    }
    
}
