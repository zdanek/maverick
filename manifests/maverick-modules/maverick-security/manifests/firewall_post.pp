class maverick-security::firewall_post {
    
    firewall { '999 drop all':
        proto   => 'all',
        action  => 'drop',
        before  => undef,
    }
  
}