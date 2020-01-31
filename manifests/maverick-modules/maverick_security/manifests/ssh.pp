# @summary
#   Maverick_security::ssh class
#   This class installs and manages SSH service and login banners.
#
# @example Declaring the class
#   This class is included from maverick_security class and should not be included from elsewhere
#
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
