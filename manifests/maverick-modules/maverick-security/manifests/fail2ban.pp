class maverick-security::fail2ban (
        $alertemail = "admin@example.com",
    ) {

    class { "::fail2ban":
        ignoreip            => hiera("all_ips"),
        destemail           => $alertemail,
        bantime             => 1800,
        maxretry            => 6,
    }
    class { "::fail2ban::jail::ssh": }

}
