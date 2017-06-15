class maverick_hardware::odroid (
    $governor_atboot = "performance",
    $kernel4x = false,
    $remove_more_packages = true,
) {

    ensure_packages(["axel", "whiptail"])

    if $remove_more_packages == true {
        ensure_packages(["chromium-browser", "thunderbird", "fonts-noto-cjk", "gnome-user-guide"], {'ensure'=>'purged'})
        if ! getvar("maverick_intelligence::tensorflow") {
            package { ["openjdk-8-jre-headless", "openjdk-8-jre"]:
                ensure      => purged
            }
        }
    }
    
    # Add odroid-utility repo from git, contains useful scripts
    oncevcsrepo { "git-odroid-utility":
        gitsource   => "https://github.com/mdrjr/odroid-utility.git",
        dest        => "/srv/maverick/software/odroid-utility",
    }
    
    # Expand rootfs
    if str2bool($::rootpart_expanded == false) and str2bool($::rootpart_device) {
        warning("Root Partition does not fill available disk, expanding.  Please reboot after this run.")
        file { "/aafirstboot":
            content     => template("maverick_hardware/aafirstboot.erb"),
            mode        => "755",
        } ->
        file { "/.first_boot":
            ensure      => present,
        }
    }
    
    # Add odroid-cpu-control from git, very useful
    oncevcsrepo { "git-odroid-cpu-control":
        gitsource   => "https://github.com/mad-ady/odroid-cpu-control.git",
        dest        => "/srv/maverick/software/odroid-cpu-control",
    } ->
    file { "/srv/maverick/software/maverick/bin/cpu-control":
        ensure      => link,
        target      => "/srv/maverick/software/odroid-cpu-control/odroid-cpu-control",
    }
    
    # Change default governor at boot
    #  .. first reset all governors except what we want
    exec { "odroid-boot-governor-others":
        command     => '/bin/sed /media/boot/boot.ini -i -r -e "/^setenv governor \"${governor_atboot}\"/! s/^(setenv governor)\s(=[^,]*)?/\# setenv governor $1/"',
        onlyif      => "/bin/grep -e '^setenv governor' /media/boot/boot.ini | /bin/grep -v '${governor_atboot}'",
    } ->
    # .. then set the requested governor
    exec { "odroid-boot-governor-requested":
        command     => "/bin/sed /media/boot/boot.ini -i -r -e 's/^# setenv governor \"${governor_atboot}\"/setenv governor \"${governor_atboot}\"/'",
        onlyif      => "/bin/egrep -e '^# setenv governor \"${governor_atboot}\"' /media/boot/boot.ini"
    }
    concat { "/etc/profile.d/maverick-path-odroid.sh":
        ensure      => present,
    }
    concat::fragment { "maverickpath-cpucontrol":
        target      => "/etc/profile.d/maverick-path-odroid.sh",
        order       => 10,
        content     => "PATH=\$PATH:/srv/maverick/software/odroid-cpu-control",
    }

    # Supress irritating kernel messages
    exec { "xu4-blacklist-mod":
        command     => "/bin/echo 'blacklist ina231_sensor' >>/etc/modprobe.d/blacklist-odroid.conf",
        unless      => "/bin/grep ina231_sensor /etc/modprobe.d/blacklist-odroid.conf",
    }
    
    # Ensure Mali GL libraries are installed
    package { "mali-x11":
        ensure      => present
    } ->
    package { "mali-fbdev":
        ensure      => absent
    }
    ensure_packages(["libgles2-mesa", "libgles2-mesa-dev"])
    
    # Add odroid-wiringpi from hardkernel github
    oncevcsrepo { "git-odroid-wiringpi":
        gitsource   => "https://github.com/hardkernel/wiringPi.git",
        dest        => "/srv/maverick/software/odroid-wiringpi",
    } ->
    exec { "compile-wiringpi":
        command     => "/srv/maverick/software/odroid-wiringpi/build >/srv/maverick/var/log/build/odroid-wiringpi.build.log 2>&1",
        cwd         => "/srv/maverick/software/odroid-wiringpi",
        creates     => "/srv/maverick/software/odroid-wiringpi/wiringPi/libwiringPi.so.2.0",
    }
    
    if $kernel4x == true and $::odroid_kernel_install_flag == "no" {
        class { "maverick_hardware::odroid::kernel4x": }
    } elsif $kernel4x == true and $::odroid_kernel_install_flag == "yes" {
        # do nothing
    } else {
        class { "maverick_hardware::odroid::kernel3x": }
    }
    
}