class maverick-security (
	$selinux = "permissive",
	$fail2ban = false,
	$firewall = true,
	) {
    
    ### Turn selinux on to at least permissive by default
    if ($operatingsystem == "CentOS") or ($operatingsystem == "RedHat") or ($operatingsystem == "Fedora") {
	    class { "selinux":
			mode => $selinux,
	    }
    }
    
    ### Configure firewall.  More rules are set with each applicaton
	if $firewall {
		class { "maverick-security::firewall": }
	}

	### Configure fail2ban for ssh
	if $fail2ban {
		class { "maverick-security::fail2ban": }
	} else {
		service { "fail2ban":
			ensure		=> stopped,
			enable		=> false
		}
	}

	### Configure scanners like rkhunter and clamav
	class { "maverick-security::scanners": }
	
	### Configure/enable ssh client, including various mandatory keys
	class { "maverick-security::ssh": }
	
}