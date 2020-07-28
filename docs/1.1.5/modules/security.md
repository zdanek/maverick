# Security Module
Although Drones/UAVs are mostly used for hobbies or simple commercial work, in the future they will be more and more an integral technology.  Because of the potential danger of drones, there should be at least some effort to minimise the possibility of hacking or interfering with them.

Maverick contains several basic security measures, most of which are not enabled by default:
- Firewall
- Brute-force attack protection
- Rootkit detection
- Anti-virus

## Firewall
The firewall is enabled by default on all platforms.  This protects the system from exposing unwanted network services (and provides some protection to mitigate hacked computers).  Maverick automatically 'punches holes' in the firewall for legitimate services.  To disable the firewall, set localconf parameter:  
`"maverick_security::firewall": false`  
By default all IPs are allowed on the ports allowed by the firewall.  To restrict allowed IPs to these ports, set a localconf parameter array:  
`"firewall_ips": ["192.168.1.0", "10.0.0.1", "my.spinny.flying.thing"]`  

## Brute-force attack protection
Brute-force attacks such as dictionary attacks, are very common on connected systems.  Typically a network-connected computer will have tens to hundreds of thousands of brute-force attacks per day, attempting to throw dictionaries of known usernames and passwords at a system in the hope that someone has been careless setting their password.  The best defence against these attacks is to detect them and drop any subsequent attempts.

Maverick optionally provides 'fail2ban', which is a system that scans incoming connections and blocks subsequent attempts if certain conditions are met (for example, 5 failed attempts within 1 minute), for a certain period of time.  This greatly mitigates the effectiveness of brute-force attempts.  To enable, set the localconf parameter:  
`"maverick_security::fail2ban": true`  
The alert email address can also be set:
`"maverick_security::fail2ban::alertemail": "admin@example.com"`  
Note that currently only ssh is protected by fail2ban.  In the future this may be extended to other network services.

## Scanners
### Rootkit Scanner
Maverick can optionally install a rootkit scanner called rkhunter.  Often if the system is hacked, certain system resources will be changed - for example new backdoor users will be added, critical system binaries altered, hidden backdoor services started.  rkhunter attempts to detect these alterations and alert them for investigation.  To enable rkhunter, set localconf parameter:  
`"maverick_security::rkhunter": true`  
### Anti-virus
Maverick can optionally install an antivirus scanner called ClamAV.  To enable this, set localconf parameter:  
`"maverick_security::clamav": true`  
Note that like most antivirus software, this can use considerable resources and impact performance.
