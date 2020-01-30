# @summary
#   Base::Puppet class
#   This class installs/manages the puppet client.
#
# @example Declaring the class
#   This class is included from base class and should not be included from elsewhere
#
class base::puppet (
) {

    # Ensure system puppet is not installed, Maverick uses puppet from gems
    ensure_packages(["puppet", "facter", "hiera", "puppet-agent", "puppet-common"], {'ensure'=>'absent'})

    # Ensure puppet services are stopped, maverick runs puppet entirely in oneshot/masterless mode
	service {"puppet":
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
        content     => template("base/puppet-client.conf.erb"),
        mode        => "644",
        owner       => "root",
        group       => "root",
    }

}
