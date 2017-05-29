#!/bin/bash

# Set defaults, can be overriden in /srv/maverick/data/config/vision/vision_seek.conf
FPS=
FFC=
SCALE=
COLORMAP=
ROTATE=
CAMTYPE=seek
OUTPUT=window

# Override values from config file
[ ! -r /srv/maverick/data/config/vision/vision_seek.conf ] || . /srv/maverick/data/config/vision/vision_seek.conf

# Parse config options
if [ ! -z $FPS ]; then
    _FPS="--fps=$FPS"
else
    _FPS=""
fi
if [ ! -z $FFC ]; then
    _FFC="-FFC $FFC"
fi
if [ ! -z $SCALE ]; then
    _SCALE="--scale=$SCALE"
else
    _SCALE=""
fi
if [ ! -z $COLORMAP ]; then
    _COLORMAP="--colormap=$COLORMAP"
else
    _COLORMAP=""
fi
if [ ! -z $ROTATE ]; then
    _ROTATE="--rotate=$ROTATE"
else
    _ROTATE=""
fi
if [ ! -z $CAMTYPE ]; then
    _CAMTYPE="--camtype=$CAMTYPE"
else
    _CAMTYPE=""
fi
if [ ! -z "$OUTPUT" ]; then
    _OUTPUT="-o \"$OUTPUT\""
else
    _OUTPUT=""
fi

# Call vision_seek with configured arguments
/srv/maverick/software/libseek-thermal/bin/seek_viewer $_FPS $_SCALE $_COLORMAP $_ROTATE $_CAMTYPE $FFC -o "$OUTPUT"
