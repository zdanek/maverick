class base::sysctl {
    
  define conf ( $value ) {

    # $name is provided by define invocation
    $key = $name

     augeas { "sysctl_conf/$key":
       #context => "/files/etc/sysctl.conf",
       onlyif  => "get /files/etc/sysctl.conf/$key != '$value'",
       changes => "set /files/etc/sysctl.conf/$key '$value'",
       notify  => Exec["sysctl"],
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