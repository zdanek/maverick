# Note this shouldn't be used normally on debian/ubuntu if resolvconf is used.
# Instead, add 'nameservers' parameter to interface definitions
class maverick-network::dnsclient (
        $servers = ['127.0.0.1', '8.8.8.8', '8.8.4.4', '8.26.56.26', '8.20.247.20', '209.244.0.3', '209.244.0.4'],
        $domain = "home",
        $search_domains = ["home", "local"]
    ) {

    class { "::dnsclient":
        nameservers     => $servers,
        domain          => $domain,
        search          => $search_domains,
    }

}