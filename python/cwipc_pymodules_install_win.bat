set bindir=%~dp0
cd %bindir%\..\share\cwipc_util\python
python3 setup.py install
cd %bindir%\..\share\cwipc_realsense2\python
python3 setup.py install
cd %bindir%\..\share\cwipc_codec\python
python3 setup.py install
