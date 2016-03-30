class maverick-baremetal::beagle::init (
    ) {
    
    file { "/etc/modprobe.d/can-blacklist.conf":
        content     => template("maverick-baremetal/can-blacklist.conf.erb"),
        mode        => 644,
        owner       => "root",
        group       => "root",
    }

    class { "maverick-baremetal::beagle::services": }

   # Remove unnecessary packages from beagle installs to save space - space is at a premium on emmc
    package { ["aglfn", "alsa-utils", "ap-hotspot", "blt", "bmap-tools", "dmidecode", "libblas-common", "libblas3", "libbluray1", "libcdio-cdda1", "libcdio-paranoia1", "libcdio13", "libcups2", "libdc1394-22", "libdc1394-22-dev", "libdrm-exynos1", "libdrm-nouveau2", "libdrm-radeon1", "libdrm-tegra0", "libflac8", "libid3tag0", "libimobiledevice4", "libogg0", "libraw1394-11", "libraw1394-dev", "libraw1394-tools", "libsamplerate0", "libschroedinger", "libthai-data", "libthai0"]:
        ensure      => purged
    }
    
    if $beagle_rootdiskexpanded != "yes" {
        warning("Root Filesystem is being expanded, please reboot ASAP to activate new partition table")
        exec { "beagle_expand_rootfs":
            command     => "/opt/scripts/tools/grow_partition.sh",
        }
    }
    
    # If network managed, configure usb networking
    if defined(Class["::maverick-network"]) {
        network::interface { "usb0":
            enable_dhcp     => false,
            manage_order    => 30,
            auto            => true,
            method          => "static",
            ipaddress       => "192.168.7.2",
            netmask         => "255.255.255.252",
        }
    }
    
}
