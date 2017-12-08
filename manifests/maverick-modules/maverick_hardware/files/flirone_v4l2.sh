#!/bin/bash

# Start v4l2loopback devices
/sbin/modprobe v4l2loopback exclusive_caps=0,0 video_nr=1,2,3

# Start flirone device mapper
/srv/maverick/software/flirone/flirone /srv/maverick/software/flirone/palettes/Rainbow.raw
