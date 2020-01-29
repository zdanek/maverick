# @summary
#   Base::Sysctl class
#   This class manages system sysctl settings.
#
# @example Declaring the class
#   This class is included from base class and should not be included from elsewhere
#   The defined type 'conf' can be called many times from other manifests, and adds/updates a sysctl entry.  Multiple entries can be declared at once.
#   base::sysctl::conf {
#     "net.ipv6.conf.all.disable_ipv6": 					value => 1;
#     "net.ipv6.conf.default.disable_ipv6": 				value => 1;
#     "net.ipv6.conf.lo.disable_ipv6": 					value => 1;
#   }
#
class base::sysctl {
    
  define conf ( Variant[String, Integer] $value ) {

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
