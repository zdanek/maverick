class base::locale (
    $locale = "en_GB.UTF-8",
    $timezone = "Europe/London",
    $language_pack = "locales-all",
) {
    
    /*
    # Install language support if necessary
    if $operatingsystem == "Ubuntu" {
        exec { "install-language":
            command     => "/usr/bin/apt-get -y install \"${language_pack}\"",
            unless      => "/usr/bin/locale -a |grep -i ${locale}",
            before      => Class["locales"],
        }
    }
    */
    
    if $locale {
        $_locale = $locale
    } else {
        $_locale = "en_GB.UTF-8 UTF-8"
    }
    
    # Set system locale
    class { "locales":
        default_locale      => "${_locale}",
        locales             => [ "${_locale}" ],
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