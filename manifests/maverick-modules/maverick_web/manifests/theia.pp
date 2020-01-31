# @summary
#   Maverick_web::Theia class
#   This class installs and manages the Theia IDE.
#
# @example Declaring the class
#   This class is included from maverick_web class and should not be included from elsewhere
#
# @param active
#   If true, start the Theia service and enable at boot time.
# @param webport
#   TCP port to listen on for connection.
# @param basepath
#   The base path for Theia to present in filesystem explorer.
# @param password
#   Password to use for web connections.
#
class maverick_web::theia (
    Boolean $active = true,
    Integer $webport = 6789,
    String $basepath = "/srv/maverick",
    String $password = "wingman",
) {

    ensure_packages(["libx11-dev", "libxkbfile-dev"])
    if ! ("install_flag_theia" in $installflags) {
        oncevcsrepo { "git-theia":
            gitsource   => "https://github.com/eclipse-theia/theia",
            dest        => "/srv/maverick/software/theia",
        } ->
        exec { "theia-build":
            path        => ["/bin", "/usr/bin", "/opt/nodejs/bin"],
            command		=> "yarn  >/srv/maverick/var/log/build/theia.build.log 2>&1",
            cwd		    => "/srv/maverick/software/theia",
            creates     => "/srv/maverick/software/theia/examples/api-samples/lib/browser/api-samples-contribution.js",
            require     => [ Package['yarn'], Package["libx11-dev"], Package["libxkbfile-dev"] ],
            timeout		=> 0,
            user        => "mav",
        }
    }

    if defined(Class["::maverick_security"]) {
        maverick_security::firewall::firerule { "theia":
            ports       => $webport,
            ips         => lookup("firewall_ips"),
            proto       => "tcp"
        }
    }

    # Control running service
    if $active == true {
        $_ensure = running
        $_enable = true
    } else {
        $_ensure = stopped
        $_enable = false
    }
    file { "/etc/systemd/system/maverick-theia.service":
        content     => template("maverick_web/theia.service.erb"),
        owner       => "root",
        group       => "root",
        mode        => "644",
        notify      => [ Exec["maverick-systemctl-daemon-reload"], Service["maverick-theia"] ],
    } ->
    service { "maverick-theia":
        ensure      => $_ensure,
        enable      => $_enable,
    }
    
    # status.d entry
    file { "/srv/maverick/software/maverick/bin/status.d/120.web/102.theia.status":
        owner   => "mav",
        content => "theia,Theia IDE\n",
    }

}
