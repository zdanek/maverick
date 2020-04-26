# @summary
#   Maverick_web class
#   This class controls all other classes in maverick_web module.
#
# @example Declaring the class
#   This class is included from the environment manifests and is not usually included elsewhere.
#   It could be included selectively from eg. minimal environment.
#
# @param cloud9
#   If true, include the maverick_web::cloud9 class which installs and manages the Cloud9 IDE software.
# @param codeserver
#   If true, include the maverick_web::codeserver class which installs and manages the Codeserver/VsCode IDE software.
# @param theia
#   If true, include the maverick_web::theia class which installs and manages the Theia IDE software.
# @param nodejs
#   If true, install NodeJS software.  Lots of other Maverick components depend on NodeJS, this should be true.
# @param webserver
#   If true, install, start and manage a Maverick webserver.  Lots of other Maverick components depend on NodeJS, this should be true.
# @param webserver_port
#   Unencrypted webserver port to listen on.  Default for web browsers is port 80.
# @param webserver_sslport
#   Encrypted webserver port to listen on.  Default for web browsers is port 443.
# @param maverick_docs
#   If true, install Maverick documentation.
# @param ssl
#   If true, include maverick_web::ssl class which manages the SSL environment for web services.
# @param maverick_web
#   If true, include maverick_web::maverick_web class which installs and manages the Maverick-web software.
# @param maverick_api
#   If true, include maverick_web::maverick_api class which installs and manages the Maverick-api software.
# @param maverick_discovery
#   If true, include maverick_web::maverick_discovery class which installs and manages the Maverick-discovery software.
# @param server_fqdn
#   This is set to the system fqdn by default, but can be specified here.  It is used by a lot of other maverick_web classes.
# @param ssl_location
#   Location of SSL certificates for web services
# @param janus
#   If true, include maverick_web::janus class which installs and manages Janus WebRTC Gateway software
#
class maverick_web (
    Boolean $cloud9 = true,
    Boolean $codeserver = false,
    Boolean $theia = false,
    Boolean $nodejs = true,
    Boolean $webserver = true,
    Integer $webserver_port = 80,
    Integer $webserver_sslport = 443,
    Boolean $maverick_docs = true,
    Boolean $ssl = true,
    Boolean $maverick_web = true,
    Boolean $maverick_api = true,
    Boolean $maverick_discovery = true,
    String $server_fqdn = $::fqdn,
    String $ssl_location = "/srv/maverick/data/security/ssl/web",
    Boolean $janus = true,
) {
    
    # Remove deprecated maverick-web-legacy repo
    file { "/srv/maverick/software/maverick-web-legacy":
        ensure  => absent,
        force   => true,
    }

    # Create status.d directory for maverick status`
    file { "/srv/maverick/software/maverick/bin/status.d/120.web":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    } ->
    file { "/srv/maverick/software/maverick/bin/status.d/120.web/__init__":
        owner       => "mav",
        content     => "Web Services",
    }

    # Install tornado here as it is used across maverick modules
    install_python_module { "tornado":
        pkgname     => "tornado",
        ensure      => atleast,
        version     => "6.0.3",
    }

    if $nodejs == true {
        class { "maverick_web::nodejs": }
    }
    
    if $cloud9 == true {
        class { "maverick_web::cloud9": }
    }
    
    if $codeserver == true {
        class { "maverick_web::codeserver": }
    }

    if $theia == true {
        class { "maverick_web::theia": }
    }

    file { [ "/srv/maverick/data/web", "/srv/maverick/config/web", "/srv/maverick/var/log/web", "/srv/maverick/var/lib/web", ]:
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    }
    
    if $webserver == true {
        class { "maverick_web::nginx": 
            port    => $webserver_port,
            ssl_port => $webserver_sslport,
        }
        
        # Create hole in firewall for webserver
        if defined(Class["::maverick_security"]) {
            maverick_security::firewall::firerule { "webserver":
                ports       => [$webserver_port, $webserver_sslport],
                ips         => lookup("firewall_ips"),
                proto       => "tcp"
            }
        }

    }
    
    if $maverick_docs == true {
        class { "maverick_web::maverick_docs": }
    }
    
    if $ssl == true {
        class { "maverick_web::ssl": }
    }

    if $maverick_discovery == true {
        class { "maverick_web::maverick_discovery": }
    }
    
    if $maverick_api == true {
        class { "maverick_web::maverick_api": }
    }

    if $maverick_web == true {
        class { "maverick_web::maverick_web": }
    }
 
    if $janus == true {
        class { "maverick_web::janus": }
    }   
}
