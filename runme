#!/bin/bash

echo
echo 'Maverick - UAV Companion Computer Automation'
echo '--------------------------------------------'

# Check that we're root
if [[ $EUID -ne 0 ]]; then
    echo "Error: This must be run as root (sudo ./runme.sh)"
    echo
    exit 1
fi

# Define usage function
usage () {
	echo "--env=[dev|production]         puppet environment, currently dev or production"
	echo "--confirm                      this must be set to proceed, for safety"
	echo
	echo "WARNING: Maverick may make major changes to the system is it running on.  Please do not run without understanding what it does."
	echo
	exit 1
}

# Parse arguments
for i in "$@"
do
	case $i in
		--env=*)
		ENV="${i#*=}"
		shift
		;;
		--confirm)
		CONFIRM="true"
		shift
		;;
		#*)
		#usage
		#;;
	esac
done

# If environment not set to dev or production, exit
if [[ "$ENV" != "dev" && "$ENV" != "production" ]]; then 
	echo "Error: --env not set to a recognised environment (dev or production)"
	echo
	usage
fi

# If confirm not set, exit
if [ "$CONFIRM" != "true" ]; then	
	echo "Error: --confirm not set"
	echo
	usage
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
	echo
	exit 1
fi

# OK we're good to go!
echo 'Proceeding to apply Puppet manifests - please be patient, this can take a while..'
echo "Environment: ${ENV}"
puppet apply --confdir=conf --environment ${ENV} manifests
echo
echo "Maverick finished, happy flying :)"
echo

