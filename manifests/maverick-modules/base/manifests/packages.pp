class base::packages {

    # Remove large packages that come with raspbian as otherwise we don't have enough space to continue
    package { "sonic-pi":
	    ensure		=> purged
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
    ])

    if $operatingsystem == "Ubuntu" {
        ensure_packages(["linux-firmware"])
    } elsif $operatingsystem == "Debian" {
        ensure_packages(["firmware-linux", "firmware-atheros", "firmware-brcm80211", "firmware-ralink", "firmware-realtek"])
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
    package { ["upstart", "unity-greeter"]:
	    ensure		=> purged
    }

    # Remove ModeManager which conflicts with APM/Pixhawk
    package { "modemmanager":
        ensure      => purged
    }
    
    # Install python using python module
    class { "python":
        version    => 'system',
        pip        => 'present',
        dev        => 'present',
        virtualenv => 'present',
        gunicorn   => 'absent',
    } ->
    package { ["python-setuptools", "virtualenvwrapper", "python-numpy", "python3-numpy", "python-lockfile", "python-daemon"]:
        ensure      => present
    } ->
    python::pip {'pip-python-pyric':
        pkgname     => 'PyRIC',
        ensure      => present,
    }
    
}
