# @summary
#   Maverick_security::firewall class
#   This class installs and manages the network firewall.
#
# @example Declaring the class
#   This class is included from maverick_security class and should not be included from elsewhere
#
# @param $cronupdate
#   If true, this sets up an hourly system cron job that updates the firewall.  This is useful if using dynamic dns, to refresh updated IP addresses.
#
class maverick_security::firewall (
        Boolean $cronupdate = false,
    ) {

    Firewall {
        before  => Class['maverick_security::firewall_post'],
        require => Class['maverick_security::firewall_pre'],
    }

    package { ["iptables", "iptables-persistent"]:
      ensure      => present,
    }

    class { ['maverick_security::firewall_pre', 'maverick_security::firewall_post']:
    }

    class { '::firewall': }

    ### Purge any existing iptables rules
    resources { "firewall":
        purge => true,
    }

    ### Define some common rules across all servers
    include maverick_security::firewall_common

    ### If set, then define a cron to update iptables from puppet once an hour.  This is useful for dyndns.
    if $cronupdate == true {
        cron::job { "puppet-firewall":
            minute      => "0",
            hour        => "*",
            date        => "*",
            month       => "*",
            weekday     => "*",
            user        => "root",
            command     => '/bin/sleep `expr ${RANDOM} \% 600`; puppet agent --tags maverick_security::firewall -t >/var/log/puppet-firewall 2>&1'
        }
    }

    define firerule ($ports, $ips, $proto = "tcp") {
        any2array($ports).each |$port| {
            firewall { "100 allow ${name} access for ${proto}:${port} from ${ips}":
                dport       => $port,
                proto       => $proto,
                action      => 'accept',
                source      => $ips,
            }
        }
    }

}
