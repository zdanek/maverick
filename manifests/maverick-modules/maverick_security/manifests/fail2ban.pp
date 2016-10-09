class maverick_security::fail2ban (
        $alertemail = "admin@example.com",
    ) {

    class { "::fail2ban":
        ignoreip            => hiera("all_ips"),
        destemail           => $alertemail,
        bantime             => 1800,
        maxretry            => 6,
    }
    class { "::fail2ban::jail::ssh": }
    firewall { '900 fail2ban return rule':
        chain    => 'f2b-ssh',
        jump     => 'RETURN',
        proto    => 'all',
    }
    firewall { '001 f2b-ssh call':
      chain     => 'INPUT',
      jump      => "f2b-ssh",
      proto     => "tcp",
      dport     => [22],
    }
}
