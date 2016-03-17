class maverick-security (
	$selinux = "permissive", 
	) {
    
    ### Turn selinux on to at least permissive by default
    if ($operatingsystem == "CentOS") or ($operatingsystem == "RedHat") or ($operatingsystem == "Fedora") {
	    class { "selinux":
			mode => $selinux,
	    }
    }
    
    ### Mandatory - start basic iptables rules.  More rules are set with each applicaton
	class { "maverick-security::firewall": }

	### Mandatory - Configure fail2ban for ssh
	class { "maverick-security::fail2ban": }

	### Mandatory - Configure scanners like rkhunter and clamav
	class { "maverick-security::scanners": }
	
	### Configure/enable ssh client, including various mandatory keys
	class { "maverick-security::ssh": }
	
}