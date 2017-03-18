#!/bin/bash

# Set defaults, can be overriden in /srv/maverick/data/config/mavlink/mavlinkrouter-<instance>.service.conf
ENABLE=true
SCREEN_NAME=mavlink-$1

[ ! -r /srv/maverick/data/config/mavlink/mavlinkrouter-$1.service.conf ] || . /srv/maverick/data/config/mavlink/mavlinkrouter-$1.service.conf

if [ "$ENABLE" == "false" ]; then
    echo "ENABLE flag is set to false, exiting"
    exit 0
fi

/srv/maverick/software/mavlinkrouter/bin/mavlink-routerd  -c /srv/maverick/data/config/mavlink/mavlinkrouter-$1.conf
