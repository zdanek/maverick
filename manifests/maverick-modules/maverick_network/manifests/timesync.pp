# @summary
#   Maverick_network::Timesync class
#   This class installs/manages system time synchronization software/configuration.
#
# @example Declaring the class
#   This class is included from maverick_network class and should not be included from elsewhere
#
# @param servers
#   List of ntp servers to use for synchronization.
# @param active
#   Activate system time sync and enable at boot time.
# @param type
#   The type of timesync software to use.
#
class maverick_network::timesync (
        Array[String] $servers = ['0.pool.ntp.org', '1.pool.ntp.org', '2.pool.ntp.org', '3.pool.ntp.org'],
        Optional[Boolean] $active = undef,
        Optional[Enum['ntp', 'timesyncd', 'chronyd']] $type = undef,
    ) {

    if $active == false {
      if $timesync_timesyncd == "yes" {
        service { "systemd-timesyncd.service":
          enable     => false,
          ensure     => stopped,
        }
      }

      if $timesync_chronyd == "yes" {
        service { "chronyd.service":
          enable     => false,
          ensure     => stopped,
        }
      }

      if $timesync_ntp == "yes" {
        class {'::ntp':
            service_enable => false,
            service_ensure => stopped,
        }
      }
    } elsif $active == true {
      # TODO: Autodetect type based on OS version
      if $type == "ntp" {
          ### Setup NTP to point to central ntp servers
          class {'::ntp':
              servers => $servers,
              restrict => ['127.0.0.1'],
              package_ensure => 'present',
              service_enable => true,
              service_ensure => running,
          }
      } elsif $type == "timesyncd" {
        # TODO: Setup timesyncd config
        service { "systemd-timesyncd.service":
          enable     => true,
          ensure     => running,
        }
      } elsif $type == "chronyd" {
        # TODO: Setup chronyd config
        service { "chronyd":
          enable     => true,
          ensure     => running,
        }
      }
    }
}
