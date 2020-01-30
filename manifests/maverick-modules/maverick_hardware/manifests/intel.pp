# @summary
#   Maverick_hardware::Intel class
#   This class installs/manages the common Intel hardware environment
#
# @example Declaring the class
#   This class is included from various maverick_hardware classes that are based on Intel hardware.
#
# @param mraa
#   If true, install the MRAA GPIO access library.
# @param intel_graphics
#   If true, install accelerated Intel graphics support.
# @param opencl
#   If true, install Beignet OpenCL software
#
class maverick_hardware::intel (
    Boolean $mraa = true,
    Boolean $intel_graphics = true,
    Boolean $opencl = true,
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

    # Remove deprecated intel repo
    file { "/etc/apt/sources.list.d/01org-graphics.list":
        ensure => absent,
    }

    if $intel_graphics == true {
        ensure_packages(["i965-va-driver", "intel-gpu-tools", "libcairo2", "libdrm-intel1", "libdrm2", "libva-x11-2", "libvdpau-va-gl1", "vainfo", "libva-dev", "libva-drm2", "libva-glx2", "va-driver-all", "vdpau-va-driver", "mesa-va-drivers"])
    }

    ### Install beignet/opencl
    if $opencl == true {
        # Install dependencies
        ensure_packages(["cmake", "pkg-config", "ocl-icd-dev", "libegl1-mesa-dev", "ocl-icd-opencl-dev", "libdrm-dev", "libxfixes-dev", "libxext-dev", "libtinfo-dev", "libedit-dev", "zlib1g-dev", "clinfo"])
        if $::operatingsystem == "Ubuntu" and versioncmp($::operatingsystemrelease, "18.04") == 0 {
            $_packages = ["llvm-dev", "clang", "libclang-dev"]
        } elsif ($::operatingsystem == "Debian" and versioncmp($::operatingsystemmajrelease, "9") >= 0) or ($::operatingsystem == "Ubuntu" and versioncmp($::operatingsystemmajrelease, "17") == 0) {
            $_packages = ["llvm-3.8-dev", "clang-3.8", "libclang-3.8-dev"]
        } elsif $::operatingsystem == "Ubuntu" and versioncmp($::operatingsystemrelease, "16.04") == 0 {
            $_packages = ["llvm-3.6-dev", "clang-3.6", "libclang-3.6-dev"]
        } else {
            $_packages = []
        }
        if ! ("install_flag_beignet" in $installflags and !empty($_packages)) {
            ensure_packages($_packages, {'before'=>Exec["beignet-prepbuild"]})
            # Clone and Build beignet
            oncevcsrepo { "git-beignet":
                gitsource   => "https://github.com/intel/beignet.git",
                dest        => "/srv/maverick/var/build/beignet",
                revision    => "master",
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
