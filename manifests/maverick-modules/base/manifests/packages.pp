class base::packages {

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
    }
    
    # These packages should be removed from all installs.  
    # Usually these are included in the default OS build and are not necessary, or cause some conflict or unwanted behaviour
    # We have to list them independently because putting them in a package [] doesn't seem to deal with dependencies
    if ($operatingsystem == "CentOS") or ($operatingsystem == "Fedora") or ($operatingsystem == "RedHat") {
        package { "fprintd-pam": ensure => absent } ->
        package { "fprintd": ensure => absent } ->
        package { "libfprint": ensure => absent } 
    }

    # Remove upstart as it breaks ubuntu which is now systemd
    # This is done in maverick shell script but make sure here
    #package { ["upstart", "unity-greeter"]:
	#    ensure		=> purged
    #}

    # Remove ModeManager which conflicts with APM/Pixhawk
    package { "modemmanager":
        ensure      => purged
    }
    
    # Install python using python module
    class { "python":
        version    => 'system',
        dev        => 'present',
        virtualenv => 'present',
        gunicorn   => 'absent',
    } ->
    package { ["python-setuptools", "virtualenvwrapper", "python-numpy", "python3-numpy", "python-lockfile", "python-daemon"]:
        ensure      => present
    } ->

    # Need to install/upgrade pip to a known version using easy_install, which is the only method that works reliably.
    exec { "upgrade-pip":
        command     => "sudo easy_install -U pip==9.0.1",
        # unless      => "pip --version |grep 9.0.1",
        creates     => "/usr/local/lib/python2.7/dist-packages/pip-9.0.1.dist-info",
        path        => ["/usr/local/bin", "/usr/bin", "/bin"],
    } ->
    # Install PyRIC and netifaces, python modules necessary to run maverick --netinfo
    install_python_module { 'pip-pyric':
        pkgname     => 'PyRIC',
        ensure      => present,
    } ->
    install_python_module { 'pip-netifaces':
        pkgname     => 'netifaces',
        ensure      => present,
    } ->
    install_python_module { 'pip-future':
        pkgname     => 'future',
        ensure      => present,
    }
    
    # Disable unattended upgrades
    package { "unattended-upgrades":
        ensure      => purged,
    }

    # Remove popularity-contest, https://github.com/fnoop/maverick/issues/503
    package { "popularity-contest":
        ensure      => absent,
    }
    
    # Add ruby-dev, needed for some gems to update/install
    ensure_packages(["ruby-dev"])
}
