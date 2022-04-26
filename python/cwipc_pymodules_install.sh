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
set -x
"$python3" -m pip --quiet install importlib.metadata
"$python3" -m pip --quiet uninstall --yes cwipc_util cwipc_codec cwipc_realsense2 cwipc_codec
"$python3" -m pip --quiet install --upgrade --find-links="$sharedir/cwipc/python" cwipc_util cwipc_codec cwipc_realsense2 cwipc_codec
#
# Finally, link python-based programs into bin
#
if [ -d "$installdir/bin" -a -d "$installdir/python37embedded/Scripts" ]; then
    ln -f "$installdir/python37embedded/Scripts/cwipc_"* "$installdir/bin/"
fi