$bindir = $PSScriptRoot

python -m pip install importlib.metadata
python -m pip install --upgrade --find-links="$bindir\..\share\cwipc\python" cwipc_util cwipc_codec cwipc_realsense2 cwipc_kinect
