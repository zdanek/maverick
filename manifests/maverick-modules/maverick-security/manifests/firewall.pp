class maverick-security::firewall (
        $cronupdate = false,
    ) {
    
    ### Purge any existing iptables rules
    resources { "firewall":
        purge => true,
    }
	
	Firewall {
		before  => Class['maverick-security::firewall_post'],
		require => Class['maverick-security::firewall_pre'],
	}
	
    class { ['maverick-security::firewall_pre', 'maverick-security::firewall_post']:
    }
   	
    class { '::firewall':
    	ensure		=> running,
    	require		=> Package["cups-filters"] # ensure cups-filters is removed as it can stop iptables working
    }
    
#    service { "netfilter-persistent":
#    	ensure		=> running,
#    	enable		=> true,
#    }
   	
    ### Define some common rules across all servers
    include maverick-security::firewall_common
    
    ### If set, then define a cron to update iptables from puppet once an hour.  This is useful for dyndns.
    if $cronupdate == true {
        cron::job { "puppet-firewall":
	    	minute		=> "0",
	    	hour        => "*",
	    	date        => "*",
	    	month       => "*",
	    	weekday     => "*",
		 	user		=> "root",
	    	command		=> '/bin/sleep `expr ${RANDOM} \% 600`; /usr/bin/puppet agent --tags maverick-security::firewall -t >/var/log/puppet-firewall 2>&1'
	    }
    }

    define firerule ($ports, $ips, $proto = "tcp") {
		firewall { "100 allow ${name} access from ${ips}":
			dport       => $ports,
			proto       => $proto,
			action      => accept,
			source      => $ips,
		}
	}
    
}