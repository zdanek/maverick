class maverick_security::scanners (
    $rkhunter = false,
    $clamav = false,
    ) {

    if $clamav == true {
        class { "::rkhunter": }
    }

    if $rkhunter == true {
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
        class { "::clamav":
            manage_clamd        => true,
            manage_freshclam    => true,
            manage_user         => true,
            clamd_service_ensure    => 'stopped',
            clamd_service_enable    => false,
            freshclam_service_ensure => 'stopped',
            freshclam_service_enable => false,
        }
    }
    
}