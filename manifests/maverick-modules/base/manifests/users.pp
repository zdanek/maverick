class base::users {

    ### Setup main mav user
    group { 'mav':
        gid				=> '6789',
    }->
    user { 'mav':
      ensure           => present,
      comment          => 'Maverick user',
      gid              => '6789',
      home             => '/srv/maverick',
      password         => '$6$zY2iMr0A$PwyfxSMBZBlThmC2T1025vEnY.saig.ytFzHNkx.TjKs44Z4J8P0V0K89dGm2MYL5e1orrTrrizuPuprVRCfQ.',
      password_max_age => '99999',
      password_min_age => '0',
      shell            => '/bin/bash',
      uid              => '6789',
    } ->
    file { "/srv/maverick/.bashrc":
        content 	=> template("base/mav-bashrc.erb"),
        ensure		=> present,
        owner		=> "mav",
        group		=> "mav",
        mode		=> 644,
    } ->
    file { "/etc/sudoers.d/mav":
        content 	=> "mav ALL=(ALL) NOPASSWD: ALL",
        ensure		=> present,
        owner		=> "root",
        group		=> "root",
        mode		=> 644,
    }
    
}
