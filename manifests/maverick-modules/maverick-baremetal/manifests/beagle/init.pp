class maverick-baremetal::beagle::init (
    ) {
    
    file { "/etc/modprobe.d/can-blacklist.conf":
        content     => template("maverick-baremetal/can-blacklist.conf.erb"),
        mode        => 644,
        owner       => "root",
        group       => "root",
    }

    class { "maverick-baremetal::beagle::services": }
    
}