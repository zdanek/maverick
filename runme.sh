#!/bin/sh
puppet apply --confdir=conf --environment $1 manifests
