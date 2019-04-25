#
class collectd::service {

  assert_private()

  if $collectd::manage_service {
    service { $collectd::service_name:
      ensure => $collectd::service_ensure,
      name   => $collectd::service_name,
      enable => $collectd::service_enable,
    }
  }

}
