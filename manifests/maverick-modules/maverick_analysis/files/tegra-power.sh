#!/bin/bash

# Script to measure power consumption from Nvidia Tegra, using jtx1inst

HOSTNAME="${COLLECTD_HOSTNAME:-localhost}"
INTERVAL="${COLLECTD_INTERVAL:-10}"
 
while true
do
    POWER=$(LD_LIBRARY_PATH=/srv/maverick/software/jtx1inst/lib /srv/maverick/software/jtx1inst/bin/jtx1inst |grep '\[POWER\] module power input' |awk -F: {'print $2'} |sed 's/\..*//')
    VALUE=$(echo "scale=3;$POWER/1000" | bc)
    echo "PUTVAL \"$HOSTNAME/power/gauge-watts\" interval=$INTERVAL N:$VALUE"
    sleep $INTERVAL
done