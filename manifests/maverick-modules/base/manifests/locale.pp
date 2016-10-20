class base::locale (
    $locale = "",
    $timezone = "Europe/London",
) {
    
    if $operatingsystem == "Ubuntu" {
        ensure_packages(["language-pack-en"])
    } elsif $operatingsystem == "Debian" {
        ensure_packages(["locales-all"])
    }

    if $locale {
        $_locale = $locale
    } else {
        if $operatingsystem == "Ubuntu" {
            $_locale = "en_GB.UTF8"
        } elsif $operatingsystem == "Debian" {
            $_locale = "en_GB"
        }
    }
    
    # Set system locale
    class { "locales":
        default_locale      => "${locale}",
        locales             => [ "${locale}" ],
    }

    # Set the timezone using systemd
    exec { "tz-systemd":
        command     => "/usr/bin/timedatectl set-timezone ${timezone}",
        unless      => "/bin/grep '${timezone}' /etc/timezone",
    }
    
}