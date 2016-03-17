class maverick-security::ssh {
    
    class { "::ssh": 
        storeconfigs_enabled => false,
    }
        
}