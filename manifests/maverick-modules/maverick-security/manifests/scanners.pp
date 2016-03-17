class maverick-security::scanners (
    ) {

    class { "::rkhunter": }

    class { "::clamav":
        manage_clamd        => true,
        manage_freshclam    => true,
        manage_user         => true,
        clamd_options       => {
            'MaxScanSize'   => '200M',
            'MaxFileSize'   => '100M',
        },
    }


}