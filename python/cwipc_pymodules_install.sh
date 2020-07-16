#!/bin/sh
# Find Actual INSTALL_PREFIX
bindir=`dirname $0`
installdir=`dirname $bindir`
sharedir=$installdir/share
(cd $sharedir/cwipc_util/python ; python3 setup.py install)
(cd $sharedir/cwipc_realsense2/python ; python3 setup.py install)
(cd $sharedir/cwipc_codec/python ; python3 setup.py install)
