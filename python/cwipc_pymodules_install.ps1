$bindir = $PSScriptRoot
$topdir = Split-Path -Parent $bindir
$wheeldir = join-path $topdir "share" "cwipc" "python"
python -m pip install importlib.metadata
python -m pip install --upgrade --find-links="$wheeldir" $wheeldir/cwipc_*.whl
