class base::packages {

    # These packages are installed by default for all installs.  
    # Be careful of what is put here, usually packages should be put in more specific manifests
    package {[
        "telnet",
        "wget",
        "iotop",
        ]:
        ensure	=> installed
    }
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
    
}
