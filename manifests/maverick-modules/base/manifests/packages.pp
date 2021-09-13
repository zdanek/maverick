# @summary
#   Base::Packages class
#   This class installs/manages basic system software.
#   It strips out a bunch of software that is unlikely to ever be used on a UAV, and installs basic packages that are commonly used.
#
# @example Declaring the class
#   This class is included from base class and should not be included from elsewhere
#
# @param docker
#   If false, disables and uninstalls docker, primarily to ensure it doesn't get in the way of network setup.  This will be improved in the future.
#   If set to anything else (null, undefined, true), then no action will currently be taken.
#
class base::packages (
    $docker = false,
) {

    if $docker == false {
        service { "docker":
            ensure      => stopped,
            enable      => false,
        } ->
        package { "docker.io":
            ensure      => absent,
        } ->
        exec { "docker-remove-interface":
            command => "/sbin/ip link delete docker0",
            onlyif  => "/sbin/ip a show dev docker0",
        }
    }

    # Remove some stuff that definitely doesn't belong in a robotics build
    package { ["kodi", "kodi-bin", "kodi-data", "libreoffice", "ubuntu-mate-libreoffice-draw-icons", "libreoffice-base-core", "libreoffice-common", "libreoffice-core"]:
        ensure      => purged
    }

    # These packages are installed by default for all installs.  
    # Be careful of what is put here, usually packages should be put in more specific manifests
    ensure_packages([
        "telnet",
        "iotop",
        "build-essential",
        "xz-utils",
        "unzip",
        "wget",
        "curl",
        "ncurses-bin",
        "bsdmainutils",
        "v4l-utils",
        "cmake",
        "pkg-config",
        "usbutils",
        "lsof",
        "tcpdump",
        "whois",
        "debian-goodies",
        "apparmor-utils",
        "libusb-1.0-0",
        "libusb-1.0-0-dev",
        "gnupg2",
        "net-tools",
        "iptables-persistent",
    ])

    if $operatingsystem == "Ubuntu" {
        ensure_packages(["linux-firmware"])
    } elsif $operatingsystem == "Debian" {
        ensure_packages(["firmware-linux", "firmware-atheros", "firmware-brcm80211", "firmware-realtek"])
        if $::operatingsystemmajrelease == "9" {
            ensure_packages(["firmware-misc-nonfree", "firmware-linux-nonfree"])
        } else {
            ensure_packages(["firmware-ralink"])
        }
        ensure_packages(["software-properties-common"])
    }

    # These packages should be removed from all installs.  
    # Usually these are included in the default OS build and are not necessary, or cause some conflict or unwanted behaviour
    # We have to list them independently because putting them in a package [] doesn't seem to deal with dependencies
    if ($operatingsystem == "CentOS") or ($operatingsystem == "Fedora") or ($operatingsystem == "RedHat") {
        package { "fprintd-pam": ensure => absent } ->
        package { "fprintd": ensure => absent } ->
        package { "libfprint": ensure => absent }
    }

    # Remove ModeManager which conflicts with APM/Pixhawk
    package { "modemmanager":
        ensure      => purged
    }

    # Disable unattended upgrades
    package { "unattended-upgrades":
        ensure      => purged,
    }

    # Remove popularity-contest, https://github.com/goodrobots/maverick/issues/503
    package { "popularity-contest":
        ensure      => absent,
    }

    # Add ruby-dev, needed for some gems to update/install
    ensure_packages(["ruby-dev"])
}
