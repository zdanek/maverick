# @summary
#   Base::Sudoers class
#   This class manages system sudoers config.
#
# @note
#   This isn't part of maverick_security module because it needs to be part of bootstrap environment.
#
# @example Declaring the class
#   This class is included from base class and should not be included from elsewhere
#
# @param mav_sudopass
#   If mav sudo should ask for password or not
#
class base::sudoers (
  Boolean $mav_sudopass = false,
) {

  class { '::sudo':
    purge               => false,
    config_file_replace => false,
  }

  sudo::conf { 'mav-services-start':
    priority => 10,
    content  => 'mav ALL=(root) NOPASSWD: /bin/systemctl start maverick-*',
  }

  sudo::conf { 'mav-services-stop':
    priority => 10,
    content  => 'mav ALL=(root) NOPASSWD: /bin/systemctl stop maverick-*',
  }

  sudo::conf { 'mav-services-restart':
    priority => 10,
    content  => 'mav ALL=(root) NOPASSWD: /bin/systemctl restart maverick-*',
  }

  sudo::conf { 'mav-services-enable':
    priority => 10,
    content  => 'mav ALL=(root) NOPASSWD: /bin/systemctl enable maverick-*',
  }

  sudo::conf { 'mav-services-disable':
    priority => 10,
    content  => 'mav ALL=(root) NOPASSWD: /bin/systemctl disable maverick-*',
  }

  # If mav_sudopass is true, require a password for all sudo calls
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

}
