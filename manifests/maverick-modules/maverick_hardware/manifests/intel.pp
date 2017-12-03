class maverick_hardware::intel (
    $mraa = true,
    $intel_graphics = true,
    $opencl = true,
) {

        
    ### Install MRAA - Intel GPIO access library
    if $mraa == true and $::operatingsystem == "ubuntu" {
        ensure_resource("class", "apt")
        apt::ppa { 'ppa:mraa/mraa': 
            notify => Exec['apt_update']
        } ->
        package { ["libmraa1", "libmraa-dev", "mraa-tools", "python-mraa", "python3-mraa"]:
            ensure      => installed,
            require     => Exec['apt_update'],
        }
    }
    
    if $intel_graphics == true {
        if $::operatingsystem == "ubuntu" {
             # Install vaapi support
            exec { "01org-gfx-repo-key":
                command         => "/usr/bin/wget --no-check-certificate https://download.01.org/gfx/RPM-GPG-KEY-ilg-4 -O - | apt-key add -",
                unless          => "/usr/bin/apt-key list |/bin/grep 39B88DE4",
            } ->
            file { "/etc/apt/sources.list.d/01org-graphics.list":
                content     => "deb https://download.01.org/gfx/ubuntu/16.04/main xenial main",
                notify      => Exec["apt_update"],
            } ->
            package { ["i915-4.6.3-4.4.0-dkms"]:
                ensure          => latest,
                require         => Exec["apt_update"],
                before          => Package["i965-va-driver"],
            } ->
            # Install GL support
            #package { ["libegl1-mesa-drivers", "libgles1-mesa", "libosmesa6", "mesa-va-drivers", "libegl1-mesa", "libgl1-mesa-dri", "libgl1-mesa-glx", "libglapi-mesa", "libgles2-mesa"]:
            #    ensure          => latest,
            #} ->
            # Yes this is wierd, but it's needed for intel graphics updater.  Remove.
            package { ["fonts-ancient-scripts", "ttf-ancient-fonts" ]:
                ensure          => purged,
            }
        }
        # Install misc support
        ensure_packages(["i965-va-driver", "intel-gpu-tools", "libcairo2", "libdrm-intel1", "libdrm2", "libva-x11-1", "libvdpau-va-gl1", "vainfo", "libva1", "libva-dev", "libva-drm1", "libva-egl1", "libva-glx1", "libva-tpi1", "va-driver-all"])
    }
    
    ### Install beignet/opencl
    if $opencl == true {
        # Install dependencies
        ensure_packages(["cmake", "pkg-config", "ocl-icd-dev", "libegl1-mesa-dev", "ocl-icd-opencl-dev", "libdrm-dev", "libxfixes-dev", "libxext-dev", "libtinfo-dev", "libedit-dev", "zlib1g-dev", "clinfo"])
        if $::operatingsystem == "Debian" and $::operatingsystemmajrelease == "9" {
            $_packages = ["llvm-3.8-dev", "clang-3.8", "libclang-3.8-dev"]
        } elsif $::operatingsystem == "Ubuntu" and $::operatingsystemrelease == "16.04" {
            $_packages = ["llvm-3.6-dev", "clang-3.6", "libclang-3.6-dev"]
        }
        if ! ("install_flag_beignet" in $installflags) {
            ensure_packages($_packages, {'before'=>Exec["beignet-prepbuild"]})
            # Clone and Build beignet
            oncevcsrepo { "git-beignet":
                gitsource   => "https://github.com/intel/beignet.git",
                dest        => "/srv/maverick/var/build/beignet",
                revision    => "Release_v1.3.2",
            } ->
            # Create build directory
            file { "/srv/maverick/var/build/beignet/build":
                ensure      => directory,
                owner       => "mav",
                group       => "mav",
                mode        => "755",
            } ->
            file { ["/etc/OpenCL", "/etc/OpenCL/vendors"]:
                ensure      => directory,
                owner       => "mav",
                group       => "mav",
                mode        => "755",
            } ->
            exec { "beignet-prepbuild":
                user        => "mav",
                timeout     => 0,
                command     => "/usr/bin/cmake -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/beignet .. >/srv/maverick/var/log/build/beignet.cmake.out 2>&1",
                cwd         => "/srv/maverick/var/build/beignet/build",
                creates     => "/srv/maverick/var/build/beignet/build/Makefile",
                require     => [ Package["ocl-icd-dev"], ],
            } ->
            exec { "beignet-build":
                user        => "mav",
                timeout     => 0,
                command     => "/usr/bin/make -j1 >/srv/maverick/var/log/build/beignet.build.out 2>&1",
                cwd         => "/srv/maverick/var/build/beignet/build",
                creates     => "/srv/maverick/var/build/beignet/build/src/libcl.so",
                require     => Exec["beignet-prepbuild"],
            } ->
            exec { "beignet-install":
                user        => "mav",
                timeout     => 0,
                command     => "/usr/bin/make install >/srv/maverick/var/log/build/beignet.install.out 2>&1",
                cwd         => "/srv/maverick/var/build/beignet/build",
                creates     => "/srv/maverick/software/beignet/lib/beignet/libcl.so",
            } ->
            file { "/srv/maverick/var/build/.install_flag_beignet":
                ensure      => file,
                owner       => "mav",
            }
        }
    }

}