class maverick_security::ssh {
    
    file { "/etc/issue.net":
        ensure          => present,
        mode            => "644",
        owner           => "root",
        group           => "root",
        content         => template("maverick_security/issue.erb"),
    }
    file { "/etc/issue":
        ensure          => present,
        mode            => "644",
        owner           => "root",
        group           => "root",
        content         => template("maverick_security/issue.erb"),
    }
    class { "::ssh": 
        storeconfigs_enabled    => false,
        server_options          => {
            'PasswordAuthentication' => 'yes',
            'AllowTcpForwarding' => 'yes',
            'X11Forwarding' => 'yes',
            'Banner' => '/etc/issue.net',
            'PrintMotd' => 'no',
        }
    }
        
}
