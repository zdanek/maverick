class maverick_puppet::client (
    $puppetlabs = false,
) {

    # If we are on a recognised/named OS, install the latest Puppet PC1
    if $::lsbdistcodename and $puppetlabs {
        # dpkg/apt can't install from a remote URL, so we have to use an exec
        exec { "install-puppetlabs-pc1":
            cwd         => "/var/tmp",
            command     => "/usr/bin/wget https://apt.puppetlabs.com/puppetlabs-release-pc1-${::lsbdistcodename}.deb; /usr/bin/dpkg --force-confold -i puppetlabs-release-pc1-${::lsbdistcodename}.deb; /usr/bin/apt update" ,
            unless      => "/usr/bin/dpkg -s puppetlabs-release-pc1",
        } ->
        package { ["puppet", "puppet-common"]:
            ensure      => purged
        } ->
        package { "puppet-agent":
            ensure      => installed
        }
    } else {
        ensure_packages(["puppet", "facter", "hiera"])
    }

    # Ensure puppet services are stopped, maverick runs puppet entirely in oneshot/masterless mode
	service {["puppet", "puppetmaster"]:
        ensure		=> "stopped",
		enable		=> false,
	}
	
    if ($operatingsystem == "Debian" or $operatingsystem == "Ubuntu") {
        package {["ruby-json"]:
            ensure	=> installed
        }
    }

    file { "/etc/puppet/puppet.conf":
        ensure      => file,
        content     => template("maverick_puppet/puppet-client.conf.erb"),
        mode        => "644",
        owner       => "root",
        group       => "root",
    }

}
