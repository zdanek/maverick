class base::locale (
    $locale = "en_GB.UTF-8",
    $timezone = "Europe/London",
) {
    
    # Set system locale
    ensure_packages(["language-pack-en"])
    class { "locales":
        default_locale      => "${locale}",
        locales             => [ "${locale}" ],
        require             => Package["language-pack-en"],
    }
    
    # Set the timezone using systemd
    exec { "tz-systemd":
        command     => "/usr/bin/timedatectl set-timezone ${timezone}",
        unless      => "/bin/grep '${timezone}' /etc/timezone",
    }
    
}