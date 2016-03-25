class base::defaults {

    Package {
        ensure 	=> "installed",
        allow_virtual => false,
    }
    
    Exec	{ 
        path		=> '/usr/bin:/usr/sbin:/bin:/sbin',
    }
    
    # Fix for broken upstart service provider on odroid ubuntu
    if $odroid_present and $operatingsystem == "Ubuntu" and $operatingsystemrelease == "15.10" {
        Service {
            provider    => "systemd"
        }
    }
    
    # Activate puppetlabs-stdlib at the first opportunity
    class { "stdlib": }

}
