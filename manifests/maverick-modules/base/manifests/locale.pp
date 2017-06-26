class base::locale (
    $locale = "en_GB.utf8",
    $timezone = "Europe/London",
    $language_pack = "locales-all",
) {
    
    #ensure_packages([$locales_package])
    #package { $locales_package:
    #    ensure      => installed,
    #    before      => Class["locales"]
    #}
    
    # Install language support if necessary
    exec { "install-language":
        command     => "/usr/bin/apt-get -y install \"${language_pack}\"",
        unless      => "/usr/bin/locale -a |grep -i ${locale}",
        before      => Class["locales"],
    }
    
    if $locale {
        $_locale = $locale
    } else {
        if $operatingsystem == "Ubuntu" {
            $_locale = "en_GB.utf8"
        } elsif $operatingsystem == "Debian" {
            $_locale = "en_GB"
        }
    }
    
    # Set system locale
    class { "locales":
        default_locale      => "${_locale}",
        locales             => [ "en_GB.utf8", "${_locale}" ],
    }

    # Set the timezone using systemd
    exec { "tz-systemd":
        command     => "/usr/bin/timedatectl set-timezone ${timezone}",
        unless      => "/bin/grep '${timezone}' /etc/timezone",
    }
    
    # Set TZ environment variable, https://github.com/fnoop/maverick/issues/497
    file { "/etc/profile.d/01-TZ.sh":
        content     => 'export TZ=":/etc/localtime"',
        mode        => "644",
    }
    
}