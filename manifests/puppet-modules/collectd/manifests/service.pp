#
class collectd::service {
  assert_private()

  if $collectd::manage_service {
    service { 'managed_collectd':
      ensure => $collectd::service_ensure,
      name   => $collectd::service_name,
      enable => $collectd::service_enable,
    }
  }
}
