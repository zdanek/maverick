class maverick_security::fail2ban (
        $alertemail = "admin@example.com",
    ) {

    class { "::fail2ban":
        ignoreip            => hiera("firewall_ips"),
        destemail           => $alertemail,
        bantime             => 1800,
        maxretry            => 6,
    }
    class { "::fail2ban::jail::ssh": }

}
