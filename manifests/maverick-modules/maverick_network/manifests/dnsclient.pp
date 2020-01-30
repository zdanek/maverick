# @summary
#   Maverick_network::Dnsclient class
#   This class installs/manages /etc/resolv.conf network dns client config.
#
# @note
#   This shouldn't be used normally on debian/ubuntu if resolvconf is used.
#   Instead, add 'nameservers' parameter to interface definitions.
#
# @example Declaring the class
#   This class is included from maverick_network class and should not be included from elsewhere
#
# @param servers
#   List of DNS servers to use for dns resolution.
# @param domain
#   Default domain that this system belongs to.
# @param search_domains
#   List of domains that are suffixed to hostnames if a fully qualified lookup does not return.
#
class maverick_network::dnsclient (
        Array[String] $servers = ['127.0.0.1', '8.8.8.8', '8.8.4.4', '8.26.56.26', '8.20.247.20', '209.244.0.3', '209.244.0.4'],
        String $domain = "home",
        Array[String] $search_domains = ["home", "local"]
    ) {

    class { "::dnsclient":
        nameservers     => $servers,
        domain          => $domain,
        search          => $search_domains,
    }

}
