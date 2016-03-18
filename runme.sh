#!/bin/bash

# Check that we're root
if [[ $EUID -ne 0 ]]; then
    echo "Error: This must be run as root" 1>&2
    exit 1
fi

# Check that puppet is installed
if ! hash puppet 2>/dev/null; then
	echo 'Puppet not installed, attempting to install..'
	if hash apt-get 2>/dev/null; then
		DEBIAN_FRONTEND=noninteractive apt-get -y install puppet >/dev/null 2>&1
		if hash puppet; then
			puppetinstalled=true
		fi
	fi
else
	puppetinstalled=true
fi
if ! $puppetinstalled; then
	echo 'Error: Puppet not installed and could not be installed'
	exit 1
fi

# OK we're good to go!
puppet apply --confdir=conf --environment $1 manifests
