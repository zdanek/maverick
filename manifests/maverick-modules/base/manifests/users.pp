class base::users (
    $mav_password = '$6$YuXyoBZR$cR/cNLGZV.Y/nfW6rvK//fjnr84kckI1HM0fhPnJ3MVVlsl7UxaK8vSw.bM4vTlkF4RTbOSAdi36c5d2hJ9Gj1',
    $mav_sudopass = false,
    $root_password = '$6$MIBUXpXc$AA8j.88LvHFBzvVKofKcHnEqvWdv5Cl5D8.O8aB446Mao2X4UkuJ.1VKSr2VcmsbZB7A5ypmmkO0MWGAZr37N.',
) {

    ### Setup main mav user
    group { 'mav':
        gid				=> '6789',
    }->
    user { 'mav':
      ensure           => present,
      comment          => 'Maverick user',
      gid              => '6789',
      home             => '/srv/maverick',
      password         => "${mav_password}",
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
    } 
    if $mav_sudopass == false {
        file { "/etc/sudoers.d/mav":
            content 	=> "mav ALL=(ALL) NOPASSWD: ALL",
            ensure		=> present,
            owner		=> "root",
            group		=> "root",
            mode		=> 644,
        }
    } else {
        file { "/etc/sudoers.d/mav":
            content 	=> "mav ALL=(ALL) ALL",
            ensure		=> present,
            owner		=> "root",
            group		=> "root",
            mode		=> 644,
        }
    }
    
    user { "root":
        password    => "${root_password}",
    }
}
