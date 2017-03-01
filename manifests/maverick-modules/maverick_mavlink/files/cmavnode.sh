#!/bin/bash

# Set defaults, can be overriden in /srv/maverick/data/config/mavlink/cmavnode-<instance>.service.conf
ENABLE=true
SCREEN_NAME=mavlink-$1

[ ! -r /srv/maverick/data/config/mavlink/cmavnode-$1.service.conf ] || . /srv/maverick/data/config/mavlink/cmavnode-$1.service.conf

if [ "$ENABLE" == "false" ]; then
    echo "ENABLE flag is set to false, exiting"
    exit 0
fi

/usr/bin/screen -L -c /srv/maverick/data/config/mavlink/cmavnode-$1.screen.conf -S $SCREEN_NAME -d -m /srv/maverick/software/cmavnode/bin/cmavnode -f /srv/maverick/data/config/mavlink/cmavnode-$1.conf -i -v
