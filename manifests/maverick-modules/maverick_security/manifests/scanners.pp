class maverick_security::scanners (
    $rkhunter = false,
    $clamav = false,
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