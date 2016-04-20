class maverick-baremetal::odroid::init {

    file { "/srv/maverick/software/odroid-utility":
        ensure 		=> directory,
        require		=> File["/srv/maverick/software"],
        mode		=> 755,
        owner		=> "mav",
        group		=> "mav",
    } ->
    vcsrepo { "/srv/maverick/software/odroid-utility":
        ensure		=> present,
        provider 	=> git,
        source		=> "https://github.com/mdrjr/odroid-utility.git",
        revision	=> "master",
        owner		=> "mav",
        group		=> "mav",
    }

    ensure_packages(["axel", "whiptail"])
        
    # Supress irritating kernel messages
    exec { "xu4-blacklist-mod":
        command     => "/bin/echo 'blacklist ina231_sensor' >>/etc/modprobe.d/blacklist-odroid.conf",
        unless      => "/bin/grep ina231_sensor /etc/modprobe.d/blacklist-odroid.conf",
    }
    
    # Ensure Mali GL stuff is installed
    package { "mali-x11":
        ensure      => absent
    }
    ensure_packages(["mali-fbdev"])
    
}