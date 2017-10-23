class maverick_hardware::peripheral::seekthermal (
    $libseek = false,
    $seek_id = "0010", # 0010 for Compact, 0011 for Compact Pro
) {
    
    # If Seek Thermal camera isn't detected (ie. plugged in), then use $seek_id
    if $seekthermal_modelid != "None" {
        $_seekid = $seekthermal_modelid
    } else {
        $_seekid = $seek_id
    }

    # Install udev rule so seek thermal can be used by mav user
    file { "/etc/udev/rules.d/95-seekthermal.rules":
        owner       => "root",
        group       => "root",
        mode        => "644",
        content     => "SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"289d\", ATTRS{idProduct}==\"${$_seekid}\", MODE=\"0664\", OWNER=\"mav\", GROUP=\"mav\"\n",
    }
    
    # Install libseek
    if $libseek == true {
        if ! ("install_flag_libseek" in $installflags) {
            oncevcsrepo { "git-libseek":
                gitsource   => "https://github.com/zougloub/libseek.git",
                dest        => "/srv/maverick/var/build/libseek",
            } ->
            file { "/srv/maverick/var/build/libseek/waf":
                ensure      => present,
                source      => "puppet:///modules/maverick_hardware/seek-waf",
                owner       => "mav",
                group       => "mav",
                mode        => "755",
            } ->
            file { "/srv/maverick/software/libseek":
                ensure      => directory,
                owner       => "mav",
                group       => "mav",
                mode        => "755",
            } ->
            exec { "prep-libseek":
                command     => "/srv/maverick/var/build/libseek/waf configure --prefix /srv/maverick/software/libseek",
                cwd         => "/srv/maverick/var/build/libseek",
                creates     => "/srv/maverick/var/build/libseek/build/config.log",
                user        => "mav",
            } ->
            exec { "compile-libseek":
                command     => "/srv/maverick/var/build/libseek/waf",
                cwd         => "/srv/maverick/var/build/libseek",
                creates     => "/srv/maverick/var/build/libseek/build/seek-test",
                user        => "mav",
            } ->
            exec { "install-libseek":
                command     => "/srv/maverick/var/build/libseek/waf install --destdir /",
                cwd         => "/srv/maverick/var/build/libseek",
                creates     => "/srv/maverick/software/libseek/lib/libseek.a",
                user        => "mav",
            } ->
            file { "/srv/maverick/var/build/.install_flag_libseek":
                ensure      => present,
                owner       => "mav",
                mode        => "644",
            }
        }
    }
    
}