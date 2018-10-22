#!/bin/sh

mkdir /srv/maverick/software/tbb
mkdir -p /srv/maverick/software/tbb/lib

cd /srv/maverick/var/build/tbb
cp -R include /srv/maverick/software/tbb
cp -R cmake /srv/maverick/software/tbb

platform=$(make info |grep tbb_build_prefix |awk -F= {'print $2'})
cp -R build/${platform}_release/*.so* /srv/maverick/software/tbb/lib
