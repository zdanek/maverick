# @summary
#   Maverick_security class
#   This class controls all other classes in maverick_security module.
#
# @example Declaring the class
#   This class is included from the environment manifests and is not usually included elsewhere.
#   It could be included selectively from eg. minimal environment.
#
# @param firewall
#   If true, include the maverick_security::firewall class which manages network filtering.
# @param fail2ban
#   If true, include the maverick_security::fail2ban class which manages brute force proteciton software.
# @param rkhunter
#		If true, activate the rkhunter rootkit finder sofwtare.
# @param clamav
#		If true, activate the clamav antivirus software.
# @param ssl
#		If true, include the maverick_security::ssl class which sets up the SSL environment.
# @param ldap_server
#		If true, include the maverick_security::ldap_server class which sets up the ldap server software and configuration.
# @param disable_services
#		If true, include the maverick_security::disable_services class which stop typically unnecessary services in a UAV environment.
#
class maverick_security (
	Boolean $firewall = true,
	Boolean $fail2ban = false,
	Boolean $rkhunter = false,
	Boolean $clamav = false,
	Boolean $ssl = true,
	Boolean $ldap_server = false,
	Boolean $disable_services = true,
	) {
    
	# Create data and config directory
	file { ["/srv/maverick/data/security", "/srv/maverick/config/security"]:
		ensure		=> directory,
		owner		=> "mav",
		group		=> "mav",
		mode		=> "755",
	}
	
	### Configure firewall.  More rules are set with each applicaton
	if $firewall == true {
		class { "maverick_security::firewall": }
	}

	### Configure/enable ssh client, including various mandatory keys
	class { "maverick_security::ssh": }

	if $ssl == true {
		### Configure base SSL environment
		class { "maverick_security::ssl": }
	}

	### Configure ldap client
	class { "maverick_security::ldap_client": }

	if $ldap_server == true {
		### Configure ldap server
		class { "maverick_security::ldap_server": }
	}

	### Configure fail2ban for ssh
	if $fail2ban == true {
		class { "maverick_security::fail2ban": }
	} else {
		service { "fail2ban":
			ensure		=> stopped,
			enable		=> false
		}
	}

	### Configure scanners like rkhunter and clamav
	class { "maverick_security::scanners":
		rkhunter		=> $rkhunter,
		clamav			=> $clamav,
	}

	if $disable_services == true {
		class { "maverick_security::disable_services": }
	}

}
