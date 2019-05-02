class base::locale (
    $default_locale = "en_GB.UTF-8",
    $locales = ["en_GB.UTF-8 UTF-8", "en_US.UTF-8 UTF-8"],
    $timezone = "Europe/London",
    $language_pack = "locales-all",
) {
    
    # Install language support if necessary
    if $operatingsystem == "Ubuntu" {
        ensure_packages([$language_pack], {'before'=>Class["locales"]})
        /*
        exec { "install-language":
            command     => "/usr/bin/apt-get -y install \"${language_pack}\"",
            unless      => "/usr/bin/locale -a |grep -i ${locale}",
            before      => Class["locales"],
        }
        */
    }
    
    # Set system locale
    class { "locales":
        default_locale      => $default_locale,
        locales             => $locales,
    }

    # Set the timezone using systemd
    exec { "tz-systemd":
        command     => "/usr/bin/timedatectl set-timezone ${timezone}",
        unless      => "/bin/grep '${timezone}' /etc/timezone",
    }
    
    # Set TZ environment variable, https://github.com/goodrobots/maverick/issues/497
    file { "/etc/profile.d/01-TZ.sh":
        content     => 'export TZ=":/etc/localtime"',
        mode        => "644",
    }
    
}
