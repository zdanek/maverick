class maverick_baremetal::beagle (
    $included_cloud9 = false,
    ) {
    
    file { "/etc/modprobe.d/can-blacklist.conf":
        content     => template("maverick_baremetal/can-blacklist.conf.erb"),
        mode        => 644,
        owner       => "root",
        group       => "root",
    }

    class { "maverick_baremetal::beagle::services": }

    # Remove unnecessary packages from beagle installs to save space - space is at a premium on emmc.  Lots more we can add later on.
    package { ["aglfn", "alsa-utils", "ap-hotspot", "blt", "bmap-tools", "dmidecode", "libbluray1", "libcdio-cdda1", "libcdio-paranoia1", "libcdio13", "libcups2", "libdc1394-22", "libdc1394-22-dev", "libdrm-exynos1", "libdrm-nouveau2", "libdrm-radeon1", "libdrm-tegra0", "libflac8", "libid3tag0", "libimobiledevice4", "libogg0", "libraw1394-11", "libraw1394-dev", "libraw1394-tools", "libsamplerate0", "libschroedinger", "libthai-data", "libthai0"]:
        ensure      => purged
    }
    
    if $beagle_rootdiskexpanded != "yes" {
        warning("Root Filesystem is being expanded, please reboot ASAP to activate new partition table")
        exec { "beagle_expand_rootfs":
            command     => "/opt/scripts/tools/grow_partition.sh",
        }
    }
    
    # If network managed, configure usb networking
    if defined(Class["::maverick_network"]) {
        network::interface { "usb0":
            enable_dhcp     => false,
            manage_order    => 30,
            auto            => true,
            method          => "static",
            ipaddress       => "192.168.7.2",
            netmask         => "255.255.255.252",
        }
        # Serve dhcpd on usb0 so client gets an IP address
        # First, if networking isn't being used then include a stub dnsmasq config
        if !defined(Class["::dnsmasq"]) {
            class { "maverick_network::dnsmasq": }
        }
        ::dnsmasq::conf { "beagle-usb0-dhcpd":
            ensure      => present,
            content     => template("maverick_baremetal/beagle-usb0-template.erb"),
        }
    }
    
    # If $included_cloud9 is false, remove the builtin cloud9 to save space and remove confusion
    if $included_cloud9 == false {
        package { "c9-core-installer":
            ensure      => absent,
        }
        file { "/opt/cloud9":
            ensure => absent,
            recurse => true,
            purge => true,
            force => true,
        }
        file { "/root/.c9":
            ensure => absent,
            recurse => true,
            purge => true,
            force => true,
        }
        file { "/var/lib/cloud9":
            ensure => absent,
            recurse => true,
            purge => true,
            force => true,
        }
    }
    
}
