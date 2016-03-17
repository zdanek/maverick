#!/bin/sh

puppet module --confdir ../../conf --modulepath ./ $1 $2 $3 $4
