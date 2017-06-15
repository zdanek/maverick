class base::sysctl {
    
  define conf ( $value ) {

    # $name is provided by define invocation
    $key = $name

    # Add key if it doesn't already exist
    exec {"sysctl_conf_add_$key":
        command     => "/bin/echo '$key=$value' >>/etc/sysctl.conf",
        unless      => "/bin/grep -e '^$key\s*=' /etc/sysctl.conf",
        notify      => Exec["/sbin/sysctl -p"],
    }

    # Alter key if already exists
    exec { "sysctl_conf_$key":
        command     => "/bin/sed /etc/sysctl.conf -i -r -e 's/^$key\s*=.*/$key=$value/'",
        unless      => "/bin/grep -e '^$key\s*=\s*$value' /etc/sysctl.conf",
        notify      => Exec["/sbin/sysctl -p"],
    }

  }

   file { "sysctl_conf":
      name => $operatingsystem ? {
        default => "/etc/sysctl.conf",
      },
   }

   exec { "/sbin/sysctl -p":
      alias => "sysctl",
      refreshonly => true,
      subscribe => File["sysctl_conf"],
   }
    
}