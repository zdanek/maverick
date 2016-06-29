class maverick-vision (
    $fpv = false,
    $cv = false,
    $gstreamer_installtype = "native",
    $mjpg_streamer = false,
    $visiond_state = undef,
) {
    
    # Setup standard packages for all platforms
    ensure_packages(["v4l-utils", "v4l-conf","uvcdynctrl"])
    ensure_packages(["x264"])

    # Add v4l2 python bindings
    python::pip { 'pip-v4l2':
        pkgname     => 'v4l2',
        ensure      => present,
    }
    
    # Install gstreamer
    if $gstreamer_installtype == "native" {
        ensure_packages(["libgstreamer1.0-0", "libgstreamer-plugins-base1.0-dev", "libgstreamer1.0-dev", "gstreamer1.0-alsa", "gstreamer1.0-plugins-base", "gstreamer1.0-plugins-good", "gstreamer1.0-plugins-bad", "gstreamer1.0-plugins-ugly", "libgstrtspserver-1.0-0", "gstreamer1.0-tools", "python-gst-1.0", "gir1.2-gstreamer-1.0", "gir1.2-gst-plugins-base-1.0", "gir1.2-clutter-gst-2.0", "gir1.2-gst-rtsp-server-1.0"])
        if ($raspberry_present == "yes") {
    		ensure_packages(["gstreamer1.0-omx"])
    	}
	} elsif $gstreamer_installtype == "source" {
        # If installing from source, remove packages
        package { ["libgstreamer1.0-0", "libgstreamer1.0-dev"]:
            ensure      => purged
        }
        ensure_packages(["libglib2.0-dev", "autoconf", "libtool-bin", "bison", "flex", "gtk-doc-tools", "python-gobject", "python-gobject-dev", "libx264-dev", "gobject-introspection", "libgirepository1.0-dev"])
        file { "/srv/maverick/build/gstreamer":
            ensure      => directory,
            owner       => "mav",
            group       => "mav",
            mode        => 755,
        } ->
        oncevcsrepo { "git-gstreamer_core":
            gitsource   => "https://anongit.freedesktop.org/git/gstreamer/gstreamer.git",
            dest        => "/srv/maverick/build/gstreamer/core",
        }
        oncevcsrepo { "git-gstreamer_plugins_base":
            gitsource   => "https://anongit.freedesktop.org/git/gstreamer/gst-plugins-base.git/",
            dest        => "/srv/maverick/build/gstreamer/gst-plugins-base",
        }
        oncevcsrepo { "git-gstreamer_plugins_good":
            gitsource   => "https://anongit.freedesktop.org/git/gstreamer/gst-plugins-good.git/",
            dest        => "/srv/maverick/build/gstreamer/gst-plugins-good",
        }
        oncevcsrepo { "git-gstreamer_plugins_bad":
            gitsource   => "https://anongit.freedesktop.org/git/gstreamer/gst-plugins-bad.git/",
            dest        => "/srv/maverick/build/gstreamer/gst-plugins-bad",
        }
        oncevcsrepo { "git-gstreamer_plugins_ugly":
            gitsource   => "https://anongit.freedesktop.org/git/gstreamer/gst-plugins-ugly.git/",
            dest        => "/srv/maverick/build/gstreamer/gst-plugins-ugly",
        }
        oncevcsrepo { "git-gstreamer_omx":
            gitsource   => "https://anongit.freedesktop.org/git/gstreamer/gst-omx.git/",
            dest        => "/srv/maverick/build/gstreamer/gst-omx",
        }
        oncevcsrepo { "git-gstreamer_gst_python":
            gitsource   => "https://anongit.freedesktop.org/git/gstreamer/gst-python.git/",
            dest        => "/srv/maverick/build/gstreamer/gst-python",
        }
        oncevcsrepo { "git-gstreamer_gst_rtsp_server":
            gitsource   => "https://anongit.freedesktop.org/git/gstreamer/gst-rtsp-server/",
            dest        => "/srv/maverick/build/gstreamer/gst-rtsp-server",
        }

        exec { "gstreamer_core-build":
            user        => "mav",
            timeout     => 0,
            command     => "/srv/maverick/build/gstreamer/core/autogen.sh --disable-gtk-doc --disable-docbook && /usr/bin/make -j${::processorcount} && /usr/bin/sudo /usr/bin/make install >/srv/maverick/data/logs/build/gstreamer_core.build.out 2>&1",
            cwd         => "/srv/maverick/build/gstreamer/core",
            creates     => "/usr/local/bin/gst-launch-1.0",
            require     => [ Package["libglib2.0-dev", "bison", "flex"], Oncevcsrepo["git-gstreamer_core"], Package["libgstreamer1.0-0"], Package["libgirepository1.0-dev"] ] # ensure we have all the dependencies satisfied
        }->
        exec { "gstreamer_core-build-ldconfig":
            command     => "/sbin/ldconfig",
            unless      => "/sbin/ldconfig -v |/bin/grep libgstreamer"
        } ->
        exec { "gstreamer_gst_plugins_base":
            user        => "mav",
            timeout     => 0,
            command     => "/srv/maverick/build/gstreamer/gst-plugins-base/autogen.sh --disable-gtk-doc --disable-docbook && /usr/bin/make -j${::processorcount} && /usr/bin/sudo /usr/bin/make install >/srv/maverick/data/logs/build/gstreamer_plugins_base.build.out 2>&1",
            cwd         => "/srv/maverick/build/gstreamer/gst-plugins-base",
            creates     => "/usr/local/bin/gst-play-1.0",
            require     => [ Oncevcsrepo["git-gstreamer_plugins_base"], Package["libgstreamer1.0-0"], Package["libgirepository1.0-dev"], Exec["gstreamer_core-build"] ]
        } ->
        exec { "gstreamer_gst_plugins_good":
            user        => "mav",
            timeout     => 0,
            command     => "/srv/maverick/build/gstreamer/gst-plugins-good/autogen.sh --disable-gtk-doc --disable-docbook && /usr/bin/make -j${::processorcount} && /usr/bin/sudo /usr/bin/make install >/srv/maverick/data/logs/build/gstreamer_plugins_good.build.out 2>&1",
            cwd         => "/srv/maverick/build/gstreamer/gst-plugins-good",
            creates     => "/usr/local/lib/gstreamer-1.0/libgstjpeg.so",
            require     => [ Oncevcsrepo["git-gstreamer_plugins_good"], Package["libgstreamer1.0-0"], Package["libgirepository1.0-dev"], Exec["gstreamer_gst_plugins_base"] ]
        } ->
        exec { "gstreamer_gst_plugins_bad":
            user        => "mav",
            timeout     => 0,
            command     => "/srv/maverick/build/gstreamer/gst-plugins-bad/autogen.sh --disable-gtk-doc --disable-docbook && /usr/bin/make -j${::processorcount} && /usr/bin/sudo /usr/bin/make install >/srv/maverick/data/logs/build/gstreamer_plugins_bad.build.out 2>&1",
            cwd         => "/srv/maverick/build/gstreamer/gst-plugins-bad",
            creates     => "/usr/local/lib/libgstgl-1.0.so",
            require     => [ Oncevcsrepo["git-gstreamer_plugins_bad"], Package["libgstreamer1.0-0"], Package["libgirepository1.0-dev"], Exec["gstreamer_gst_plugins_base"] ]
        } ->
        exec { "gstreamer_gst_plugins_ugly":
            user        => "mav",
            timeout     => 0,
            command     => "/srv/maverick/build/gstreamer/gst-plugins-ugly/autogen.sh --disable-gtk-doc --disable-docbook && /usr/bin/make -j${::processorcount} && /usr/bin/sudo /usr/bin/make install >/srv/maverick/data/logs/build/gstreamer_plugins_ugly.build.out 2>&1",
            cwd         => "/srv/maverick/build/gstreamer/gst-plugins-ugly",
            creates     => "/usr/local/lib/gstreamer-1.0/libgstx264.so",
            require     => [ Package["libx264-dev"], Oncevcsrepo["git-gstreamer_plugins_ugly"], Package["libgstreamer1.0-0"], Package["libgirepository1.0-dev"], Exec["gstreamer_gst_plugins_base"] ]
        }
        exec { "gstreamer_gst_python":
            user        => "mav",
            timeout     => 0,
            command     => "/srv/maverick/build/gstreamer/gst-python/autogen.sh  --disable-gtk-doc --disable-docbook && /usr/bin/make -j${::processorcount} && /usr/bin/sudo /usr/bin/make install >/srv/maverick/data/logs/build/gstreamer_gst_python.build.out 2>&1",
            cwd         => "/srv/maverick/build/gstreamer/gst-python",
            creates     => "/usr/local/lib/gstreamer-1.0/libgstpythonplugin.so",
            require     => [ Package["python-gobject-dev"], Oncevcsrepo["git-gstreamer_gst_python"], Package["libgstreamer1.0-0"], Package["libgirepository1.0-dev"], Exec["gstreamer_gst_plugins_base"] ]
        } ->
        exec { "gstreamer_gst_rtsp_server":
            user        => "mav",
            timeout     => 0,
            command     => "/srv/maverick/build/gstreamer/gst-rtsp-server/autogen.sh  --disable-gtk-doc --disable-docbook && /usr/bin/make -j${::processorcount} && /usr/bin/sudo /usr/bin/make install >/srv/maverick/data/logs/build/gstreamer_gst_rtsp_server.build.out 2>&1",
            cwd         => "/srv/maverick/build/gstreamer/gst-rtsp-server",
            creates     => "/usr/local/lib/libgstrtspserver-1.0.so",
            require     => [ Oncevcsrepo["git-gstreamer_gst_rtsp_server"], Package["libgstreamer1.0-0"], Package["libgirepository1.0-dev"], Exec["gstreamer_gst_plugins_base"] ]
        }
        exec { "gstreamer_after-build-ldconfig":
            command     => "/sbin/ldconfig",
            unless      => "/sbin/ldconfig -v |/bin/grep libgstrtspserver"
        }

        if ($raspberry_present == "yes") {
            exec { "gstreamer_gst_omx":
                user        => "mav",
                timeout     => 0,
                command     => "/srv/maverick/build/gstreamer/gst-omx/autogen.sh --with-omx-header-path=/opt/vc/include/IL --with-omx-target=rpi --disable-gtk-doc --disable-docbook && /usr/bin/make -j${::processorcount} && /usr/bin/sudo /usr/bin/make install >/srv/maverick/data/logs/build/gstreamer_omx.build.out 2>&1",
                cwd         => "/srv/maverick/build/gstreamer/gst-omx",
                creates     => "/usr/local/lib/gstreamer-1.0/libgstomx.so",
                require     => [ Oncevcsrepo["git-gstreamer_omx"], Package["libgstreamer1.0-0"], Package["libgirepository1.0-dev"], Exec["gstreamer_gst_plugins_base"] ]
            } ->
            file { "/etc/xdg":
                ensure      => directory,
            } ->
            exec { "cp-xdg-conf":
                command     => "/bin/cp /srv/maverick/build/gstreamer/gst-omx/config/rpi/gstomx.conf /etc/xdg",
                unless      => "/bin/ls /etc/xdg/gstomx.conf",
            }
        }
        
        # Export local typelib for gobject introspection
        file { "/etc/profile.d/local-gi-typelibs.sh":
            ensure      => present,
            mode        => 644,
            owner       => "root",
            group       => "root",
            content     => "export GI_TYPELIB_PATH=/usr/local/lib/girepository-1.0:/usr/lib/girepository-1.0",
        }
        file { "/etc/systemd/system/maverick-visiond.service.d":
            ensure      => directory
        } ->
        file { "/etc/systemd/system/maverick-visiond.service.d/typelib-path.conf":
            ensure      => present,
            mode        => 644,
            content     => "[Service]\nEnvironment=\"GI_TYPELIB_PATH=/usr/local/lib/girepository-1.0:/usr/lib/girepository-1.0\""
        }
    }

    if $mjpg_streamer == true  {
        class { "maverick-vision::fpv::mjpg-streamer": }
    }
    
    if $fpv == true {
        class { "maverick-vision::fpv::init": }
    }
    
    if $cv == true {
        class { "maverick-vision::cv::init": }
    }
    
    # Link maverick-visiond into central bin directory
    file { "/srv/maverick/software/maverick/bin/maverick-visiond":
        ensure      => link,
        target      => "/srv/maverick/software/maverick/manifests/maverick-modules/maverick-vision/files/maverick-visiond",
    }
    file { "/srv/maverick/data/config/maverick-visiond.conf":
        ensure      => present,
        owner       => "mav",
        group       => "mav",
        replace     => false, # initialize but don't overwrite in the future
        source      => "puppet:///modules/maverick-vision/maverick-visiond.conf",
    }
    
    # Add visiond as a service
    file { "/etc/systemd/system/maverick-visiond.service":
        content     => template("maverick-vision/maverick-visiond.service.erb"),
        owner       => "root",
        group       => "root",
        mode        => 644,
        notify      => Exec["maverick-systemctl-daemon-reload"],
    } ->
    service { "maverick-visiond":
        ensure      => $visiond_state,
        enable      => true,
    }
    
    # Punch some holes in the firewall for rtsp
    if defined(Class["::maverick-security"]) {
        maverick-security::firewall::firerule { "vision-rtsp-udp":
            ports       => [5554],
            ips         => hiera("all_ips"),
            proto       => "udp", # allow both tcp and udp for rtsp and rtp
        }
        maverick-security::firewall::firerule { "vision-rtsp-tcp":
            ports       => [5554],
            ips         => hiera("all_ips"),
            proto       => "tcp", # allow both tcp and udp for rtsp and rtp
        }
    }
    
}