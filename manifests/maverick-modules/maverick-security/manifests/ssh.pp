class maverick-security::ssh {
    
    class { "::ssh": 
        #ssh_key_ensure  => "absent",
        ssh_key_import  => false,
    }
        
}