class maverick_network::ntpclient (
        $servers = ['0.pool.ntp.org', '1.pool.ntp.org', '2.pool.ntp.org', '3.pool.ntp.org'],
    ) {

    ### Setup NTP to point to the central IPA servers
    class {'::ntp':
        servers => $servers,
        restrict => ['127.0.0.1'],
        package_ensure => 'present',
        service_enable => true,
        service_ensure => running,
    }

}