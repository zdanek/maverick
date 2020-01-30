# @summary
#   Base class
#   The Base class sets up the fundamental Maverick environment. It declares the puppet stages and defines the entire bootstrap stage,
#   and calls the other base classes in all stages.
#
# @example Declaring the class
#   This is the most fundamental base class, and is included in most environments.  It should never be included elsewhere.
#
class base {

    # Define stages
    class { "base::stages": }
        
    ###################################
    # Bootstrap stage - these are executed before everything else
    ###################################
    
    ### Setup some Puppet defaults
    class { "base::defaults":
        stage	=> "bootstrap",
    }
    
    ### Setup locale and timezones
    class { "base::locale": 
        stage   => "bootstrap",
    }
    
    ### Make sure basic packages are installed
    class { "base::packages": 
        stage   => "bootstrap",
        require => Class["base::defaults", "base::locale"],
    }

    ### Bootstrap puppet
    class { "base::puppet":
        stage   => "bootstrap",
        require => Class["base::defaults", "base::packages", "base::locale"],
    }
    
    ### Before we do anything, set our IP in /etc/hosts
    class { "base::hostip":
        stage   => "bootstrap",
        require => Class["base::defaults", "base::locale", "base::puppet"],
    }
    
    ### Setup base system users
    class { "base::users":
        stage	=> "bootstrap",
        require		=> Class["base::defaults", "base::locale"],
    }
    
    ### Setup base maverick environment
    class { "base::maverick":
        stage   => "bootstrap",
        require => Class["base::defaults", "base::locale"],
    }

    ###################################
    # End Bootstrap stage
    ###################################

    if !defined( Class['apt'] ) {
        class { "apt": }
    }

    ### Setup python
    class { "base::python": }
    
    ### Include useful functions
    class { "base::functions": }
        
    ### Setup console visual configuration
    class { "base::console": }

    ### Enable sysctl changes
    class { "base::sysctl": }
    
    ### Setup service states
	class { "base::services": }
    
    ### Virtual Machine setup
    if str2bool("$is_virtual") {
        $servertype = "Virtual Machine"
        class { "maverick_hardware": }
    ### Physical setup
    } else {
        $servertype = "Physical Computer"
        class { "maverick_hardware": }
    }
    
    ### NOTE: Hiera will also include classes, depending on hierarchy.
    ### In particular, check hiera/defaults.json which includes classes that are run on every server,
    ###  and $environment/defaults.json, which contain default classes/values for that environment.
    
}
