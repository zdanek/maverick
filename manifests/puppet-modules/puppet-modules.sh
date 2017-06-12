#!/bin/sh

sudo puppet module --confdir /srv/maverick/software/maverick/conf --environmentpath=/srv/maverick/software/maverick/conf/environments --modulepath /srv/maverick/software/maverick/manifests/puppet-modules $1 $2 $3 $4
