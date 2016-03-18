class maverick-security::scanners (
    ) {

    class { "::rkhunter": }

    class { "::clamav":
        manage_clamd        => true,
        manage_freshclam    => true,
        manage_user         => true,
        clamd_service_ensure    => 'stopped',
        freshclam_service_ensure => 'stopped',
        clamd_options       => {
            'MaxScanSize'   => '200M',
            'MaxFileSize'   => '100M',
        },
    }


}