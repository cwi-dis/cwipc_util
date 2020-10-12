#!/bin/sh
# Find Actual INSTALL_PREFIX
bindir=`dirname $0`
installdir=`dirname $bindir`
sharedir=$installdir/share
if ! python3 --version >/dev/null 2>&1 ; then
	alias python3=python
fi
(cd $sharedir/cwipc_util/python ; python3 setup.py install)
if [ -d $sharedir/cwipc_realsense2/python ]; then
	(cd $sharedir/cwipc_realsense2/python ; python3 setup.py install)
fi
if [ -d $sharedir/cwipc_kinect/python ]; then
	(cd $sharedir/cwipc_kinect/python ; python3 setup.py install)
fi
(cd $sharedir/cwipc_codec/python ; python3 setup.py install)
