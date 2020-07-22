set bindir=%~dp0
pushd %bindir%..\share\cwipc_util\python
python setup.py install
pushd %bindir%..\share\cwipc_realsense2\python
python setup.py install
pushd %bindir%..\share\cwipc_codec\python
python setup.py install
