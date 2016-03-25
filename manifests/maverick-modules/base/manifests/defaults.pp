class base::defaults {

    Package {
        ensure 	=> "installed",
        allow_virtual => false,
    }
    
    Exec	{ 
        path		=> '/usr/bin:/usr/sbin:/bin:/sbin',
    }
    
    # Activate puppetlabs-stdlib at the first opportunity
    class { "stdlib": }

}