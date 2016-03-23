class maverick-puppet::client (
    $enabled = true,
) {

    if $enabled {
        package {["puppet", "facter", "hiera"]:
            ensure	=> installed
        }
        if ($operatingsystem == "Debian" or $operatingsystem == "Ubuntu") {
            package {["ruby-json", "augeas-tools", "augeas-lenses", "ruby-augeas"]:
                ensure	=> installed
            }
        }
    }

    service {["puppet", "puppetmaster"]:
        ensure		=> "stopped",
        enable		=> false,
        require     => Package["puppet"],
    }->
    file { "/etc/puppet/puppet.conf":
        ensure      => file,
        content     => template("maverick-puppet/puppet-client.conf.erb"),
        mode        => "644",
        owner       => "root",
        group       => "root",
    }

}
