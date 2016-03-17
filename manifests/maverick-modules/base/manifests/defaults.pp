class base::defaults {

    Package {
        ensure 	=> "installed",
    }
    
    Exec	{ 
        path		=> '/usr/bin:/usr/sbin:/bin:/sbin',
    }

}