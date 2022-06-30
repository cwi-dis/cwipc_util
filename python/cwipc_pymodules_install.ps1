$bindir = Split-Path $PSScriptRoot -Parent

python -m pip --quiet install importlib.metadata
python -m pip --quiet install --upgrade --find-links="$bindir\..\share\cwipc\python" cwipc_util cwipc_codec cwipc_realsense2 cwipc_codec
