# Base::Locale class
#
# This class manages the System Locale.  It uses the en_GB (English GB dialect) locale by default, and sets Europe/London as the default timezone.
# 
# @example Declaring the class
#   This class is included from base class and should not be included from elsewhere
#
# @param default_locale Set the active default locale, in Debian/Ubuntu format.
# @param locales Any additional locales to be installed, so they can be also activated.
# @param timezone The system timezone.
# @param language_pack By default this is locales-all, which includes all the OS locales.  This can be set to specific locales but can cause problems.  If space is not a problem, leave as locales-all.
class base::locale (
    String $default_locale = "en_GB.UTF-8",
    Array[String] $locales = ["en_GB.UTF-8 UTF-8", "en_US.UTF-8 UTF-8"],
    String $timezone = "Europe/London",
    String $language_pack = "locales-all",
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
