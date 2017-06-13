class maverick_puppet::client (
    $puppetlabs = false,
) {

    # Ensure system puppet is not installed, Maverick uses puppet from gems
    ensure_packages(["puppet", "facter", "hiera", "puppet-agent", "puppet-common"], {'ensure'=>'absent'})

    # Ensure puppet services are stopped, maverick runs puppet entirely in oneshot/masterless mode
	service_wrapper {"puppet":
        ensure		=> "stopped",
		enable		=> false,
	}
	
    if ($operatingsystem == "Debian" or $operatingsystem == "Ubuntu") {
        package {["ruby-json"]:
            ensure	=> installed
        }
    }

    file { ["/etc/puppetlabs", "/etc/puppetlabs/puppet"]:
        ensure      => directory,
        mode        => "755",
        owner       => "root",
        group       => "root",
    } ->
    file { "/etc/puppetlabs/puppet/puppet.conf":
        ensure      => file,
        content     => template("maverick_puppet/puppet-client.conf.erb"),
        mode        => "644",
        owner       => "root",
        group       => "root",
    }

}
