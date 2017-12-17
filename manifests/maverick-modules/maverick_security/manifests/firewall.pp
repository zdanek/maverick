class maverick_security::firewall (
        $cronupdate = false,
    ) {
    
    ### Purge any existing iptables rules
    resources { "firewall":
        purge => true,
    }

    Firewall {
        before  => Class['maverick_security::firewall_post'],
        require => Class['maverick_security::firewall_pre'],
    }

    class { ['maverick_security::firewall_pre', 'maverick_security::firewall_post']:
    }

    class { '::firewall':
        ensure		=> running,
        require		=> Package["cups-filters"] # ensure cups-filters is removed as it can stop iptables working
    }

#    service_wrapper { "netfilter-persistent":
#    	ensure		=> running,
#    	enable		=> true,
#    }

    ### Define some common rules across all servers
    include maverick_security::firewall_common

    ### If set, then define a cron to update iptables from puppet once an hour.  This is useful for dyndns.
    if $cronupdate == true {
        cron::job { "puppet-firewall":
            minute		=> "0",
            hour        => "*",
            date        => "*",
            month       => "*",
            weekday     => "*",
            user		=> "root",
            command		=> '/bin/sleep `expr ${RANDOM} \% 600`; puppet agent --tags maverick_security::firewall -t >/var/log/puppet-firewall 2>&1'
        }
    }

    define firerule ($ports, $ips, $proto = "tcp") {
        any2array($ports).each |$port| {
            firewall { "100 allow ${name} access for ${proto}:${port} from ${ips}":
                dport       => $port,
                proto       => $proto,
                action      => accept,
                source      => $ips,
            }
        }
    }
    
}