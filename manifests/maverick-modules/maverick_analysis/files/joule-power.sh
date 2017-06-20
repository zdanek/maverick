#!/bin/bash

# Script to measure power consumption of Intel Joule module, taken from Intel Joule Thermal Management document

HOSTNAME="${COLLECTD_HOSTNAME:-localhost}"
INTERVAL="${COLLECTD_INTERVAL:-10}"
 
while true
do
    P1=$(cat /sys/class/powercap/intel-rapl/intel-rapl\:0/energy_uj) T1=$(date +%s)
    sleep $INTERVAL
    P2=$(cat /sys/class/powercap/intel-rapl/intel-rapl\:0/energy_uj) T2=$(date +%s)
    VALUE=$(echo "scale=2;($P2-$P1)/($T2-$T1)/1000000" | bc)
    echo "PUTVAL \"$HOSTNAME/power/gauge-watts\" interval=$INTERVAL N:$VALUE"

done