# @summary
#   Base::Users class
#   This class manages the Maverick 'mav' user.
#
# @example Declaring the class
#   This class is included from base class and should not be included from elsewhere
#
# @param mav_password
#   Hashed password for mav user
# @param mav_sudopass
#   If mav sudo should ask for password or not
# @param root_password
#   Hashed password for root user
# @param manage_root_password
#   If root password should be set by $root-password
#
class base::users (
    String $mav_password = '$6$YuXyoBZR$cR/cNLGZV.Y/nfW6rvK//fjnr84kckI1HM0fhPnJ3MVVlsl7UxaK8vSw.bM4vTlkF4RTbOSAdi36c5d2hJ9Gj1',
    Boolean $mav_sudopass = false,
    String $root_password = '$6$MIBUXpXc$AA8j.88LvHFBzvVKofKcHnEqvWdv5Cl5D8.O8aB446Mao2X4UkuJ.1VKSr2VcmsbZB7A5ypmmkO0MWGAZr37N.',
    Boolean $manage_root_password = false,
) {
    ensure_packages(["ruby-shadow"])
    
    if $operatingsystem == "Ubuntu" and ($operatingsystemrelease == "15.04" or $operatingsystemrelease == "15.10" or $operatingsystemrelease == "16.04" or $operatingsystemrelease == "16.10" or $operatingsystemrelease == "17.04" or $operatingsystemrelease == "17.10") {
        $_groups = ['dialout', 'video', 'input']
    } else {
        $_groups = ['dialout', 'video']
    }
    
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
      groups           => $_groups,
      provider         => useradd,
      require          => Package["ruby-shadow"],
    } ->
    file { "/srv/maverick/.bashrc":
        content 	=> template("base/mav-bashrc.erb"),
        ensure		=> present,
        owner		=> "mav",
        group		=> "mav",
        mode		=> "644",
    } 
    if $mav_sudopass == false {
        file { "/etc/sudoers.d/mav":
            content 	=> "mav ALL=(ALL) NOPASSWD: ALL",
            ensure		=> present,
            owner		=> "root",
            group		=> "root",
            mode		=> "644",
        }
    } else {
        file { "/etc/sudoers.d/mav":
            content 	=> "mav ALL=(ALL) ALL",
            ensure		=> present,
            owner		=> "root",
            group		=> "root",
            mode		=> "644",
        }
    }
    
    if $manage_root_password == true {
        user { "root":
            password    => "${root_password}",
        }
    }
        
}
