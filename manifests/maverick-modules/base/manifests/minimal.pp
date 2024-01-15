# @summary
#   Base::Minimal class
#   This class can be used instead of the usual Base class, but makes absolutely minimal changes to the system.
#   It can be used to apply just a very specific small subset of Maverick to an existing system with the most minimal changes.
#
# @example Declaring the class
#   This is the most fundamental base class, and is only included in the 'minimal' environment.  It should never be included elsewhere.
#
class base::minimal {

    # Define stages
    class { "base::stages": }

    ###################################
    # Bootstrap stage - these are executed before everything else
    ###################################

    ### Setup some Puppet defaults
    class { "base::defaults":
        stage	=> "bootstrap",
    }

    ### Bootstrap puppet
    class { "maverick_puppet::client":
        stage   => "bootstrap",
        require => Class["base::defaults"],
    }

    ### Setup base system users
    class { "base::users":
        stage	=> "bootstrap",
        require		=> Class["base::defaults"],
    }

    ### Setup base maverick environment
    class { "base::maverick":
        stage   => "bootstrap",
        require => Class["base::defaults"],
    }

    ###################################
    # End Bootstrap stage
    ###################################

    ### Include useful functions
    class { "base::functions": }

    ### Enable sysctl changes
    class { "base::sysctl": }

    ### NOTE: Hiera will also include classes, depending on hierarchy.
    ### In particular, check hiera/defaults.json which includes classes that are run on every server,
    ###  and $environment/defaults.json, which contain default classes/values for that environment.

}
