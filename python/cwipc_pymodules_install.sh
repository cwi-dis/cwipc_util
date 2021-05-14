#!/bin/sh
# Find Actual INSTALL_PREFIX
bindir=`dirname $0`
bindir=`cd $bindir ; pwd`
installdir=`cd $bindir/.. ; pwd`
sharedir="$installdir/share"
# Determine Python to use: python3 on linux/osx, python on windows, or embeddable python
python3=python3
if ! python3 --version >/dev/null 2>&1 ; then
	python3=python
fi
if [ -f "$installdir/python37embedded/python.exe" ]; then
	python3="$installdir/python37embedded/python.exe"
	# Workaround for embedded Python: add . to pythonpath
	echo 'import sys ; sys.path.insert(0, "")' >> "$installdir/python37embedded/Lib/site-packages/sitecustomize.py"
fi
"$python3" -m pip install importlib.metadata
"$python3" -m pip  install "$sharedir/cwipc_util/python"
if [ -d "$sharedir/cwipc_realsense2/python" ]; then
	"$python3" -m pip install "$sharedir/cwipc_realsense2/python"
fi
if [ -d "$sharedir/cwipc_kinect/python" ]; then
	"$python3" -m pip install "$sharedir/cwipc_kinect/python"
fi
"$python3" -m pip install "$sharedir/cwipc_codec/python"
#
# Finally, link python-based programs into bin
#
if [ -d "$installdir/bin" -a -d "$installdir/python37embedded/Scripts" ]; then
    ln -f "$installdir/python37embedded/Scripts/cwipc_"* "$installdir/bin/"
fi