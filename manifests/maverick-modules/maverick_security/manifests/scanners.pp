# @summary
#   Maverick_security::scanners class
#   This class installs and manages security scanners.
#
# @example Declaring the class
#   This class is included from maverick_security class and should not be included from elsewhere
#
# @param rkhunter
#   If true, setup and activate rkhunter, a rootkit hunter.  This monitors the filesystem and process table for changes and suspicious processes.
# @param clamav
#   If true, setup and run clamav.  Note that clamav can take a reasonable amount of resources.
# 
class maverick_security::scanners (
    Boolean $rkhunter = false,
    Boolean $clamav = false,
    ) {

    if $rkhunter == true {
        class { "::rkhunter": }
    }

    if $clamav == true {
        class { "::clamav":
            manage_clamd        => true,
            manage_freshclam    => true,
            manage_user         => true,
            clamd_service_ensure    => 'running',
            clamd_service_enable    => true,
            freshclam_service_ensure => 'running',
            freshclam_service_enable => true,
            clamd_options       => {
                'MaxScanSize'   => '200M',
                'MaxFileSize'   => '100M',
            },
        }
    } else {
        #class { "::clamav":
        #    manage_clamd        => false,
        #    manage_freshclam    => false,
        #    manage_user         => false,
        #    clamd_service_ensure    => 'stopped',
        #    clamd_service_enable    => false,
        #    freshclam_service_ensure => 'stopped',
        #    freshclam_service_enable => false,
        #}
        service { ["clamav-freshclam", "clamd"]:
            ensure      => stopped,
            enable      => false,
        } ->
        package { ["clamav", "clamav-base", "clamav-freshclam"]:
            ensure      => purged,
        }
        
    }
    
}
