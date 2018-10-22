#!/bin/bash
export TBBROOT="/srv/maverick/software/tbb" #
if [ -z "$CPATH" ]; then #
    export CPATH="${TBBROOT}/include" #
else #
    export CPATH="${TBBROOT}/include:$CPATH" #
fi #
if [ -z "$LIBRARY_PATH" ]; then #
    export LIBRARY_PATH="${TBBROOT}/lib" #
else #
    export LIBRARY_PATH="${TBBROOT}/lib:$LIBRARY_PATH" #
fi #
if [ -z "$LD_LIBRARY_PATH" ]; then #
    export LD_LIBRARY_PATH="${TBBROOT}/lib" #
else #
    export LD_LIBRARY_PATH="${TBBROOT}/lib:$LD_LIBRARY_PATH" #
fi #
