# @summary
#   Maverick_web::Apache class
#   This class installs and manages the Apache webserver.
#
# @example Declaring the class
#   This class is included from maverick_web class and should not be included from elsewhere
#
# @param port
#   Unencrypted webserver port to listen on.  Default for web browsers is port 80.
# @param ssl_port
#   Encrypted webserver port to listen on.  Default for web browsers is port 443.
# @param server_hostname
#   Server FQDN to use to create default vhost.
# 
class maverick_web::apache (
    Integer $port = 80,
    Integer $ssl_port = 443,
    String $server_hostname = $maverick_web::server_fqdn,
) {
    
    service { "nginx":
        ensure      => stopped,
        enable      => false,
    } ->
    class { 'apache':
        default_vhost => false,
    } ->
    apache::vhost { $server_hostname:
        port        => $port,
        docroot     => '/srv/maverick/software/maverick-web-legacy/public',
        require     => Class["maverick_web::maverick_web_legacy"],
    }
    
}
