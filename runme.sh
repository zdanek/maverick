#!/bin/bash

# Check that we're root
if [[ $EUID -ne 0 ]]; then
    echo "Error: This must be run as root" 1>&2
    exit 1
fi

puppet apply --confdir=conf --environment $1 manifests
