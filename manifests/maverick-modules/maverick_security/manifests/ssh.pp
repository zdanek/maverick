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
        sshd_config_print_motd      => 'no',
        sshd_config_banner          => "/etc/issue.net",
        permit_root_login           => 'yes',
        ssh_config_forward_x11_trusted  => 'yes',
        sshd_x11_forwarding         => 'yes',
        sshd_password_authentication    => 'yes',
        ssh_key_import              => false,
        ssh_key_export              => false,
    }
        
}