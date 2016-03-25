class base::packages {

    # These packages are installed by default for all installs.  
    # Be careful of what is put here, usually packages should be put in more specific manifests
    ensure_packages([
        "telnet",
        "wget",
        "iotop",
        "python",
        "python-dev",
        "python-pip",
        "build-essential",
        "xz-utils",
        "unzip",
        "wget",
        "curl",
    ])
    if ($operatingsystem == "Debian") {
        package {[
        ]:
        ensure	=> installed
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

    # Remove large packages that come with raspbian as otherwise we don't have enough space to continue
    package { "sonic-pi":
	ensure		=> purged
    }
    # Remove upstart as it breaks ubuntu which is now systemd
    package { ["upstart", "unity-greeter"]:
	ensure		=> purged
    }
    
}
