# @summary
#   Maverick_hardware::Up class
#   This class installs/manages the Aaeon Up hardware environment
#
# @example Declaring the class
#   This class is included from maverick_hardware class and should not be included from elsewhere
#
# @param swapsize
#   Set the swap size using file.
#
class maverick_hardware::up (
    Integer $swapsize = 1024,
) {
 
    # Include Intel platform manifest
    class { "maverick_hardware::intel": }

    # A big chunk of memory stolen by shared graphics RAM, so create a swapfile
    ensure_packages(["util-linux"]) # contains swapon
    exec { "create-swap":
        command     => "/bin/dd if=/dev/zero of=/var/swap bs=1M count=${swapsize}; mkswap /var/swap",
        creates     => "/var/swap",
    } ->
    file { "/var/swap":
        mode        => "600",
    } ->
    exec { "fstab-swap":
        command     => "/bin/echo '/var/swap swap swap defaults 0 0' >>/etc/fstab",
        unless      => "/bin/grep '/var/swap' /etc/fstab",
    } ->
    exec { "swapon":
        command     => "/sbin/swapon -a",
        unless      => "/sbin/swapon -s |grep '/var/swap'",
    }
    
}
