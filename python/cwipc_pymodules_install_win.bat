@echo on
set "params=%*"
cd /d "%~dp0" && ( if exist "%temp%\getadmin.vbs" del "%temp%\getadmin.vbs" ) && fsutil dirty query %systemdrive% 1>nul 2>nul || (  echo Set UAC = CreateObject^("Shell.Application"^) : UAC.ShellExecute "cmd.exe", "/k cd ""%~sdp0"" && %~s0 %params%", "", "runas", 1 >> "%temp%\getadmin.vbs" && "%temp%\getadmin.vbs" && exit /B )

set bindir=%~dp0
set python=python
if exist "%bindir%..\python37embedded\python.exe" (
	set python="%bindir%..\python37embedded\python.exe"
	echo "import sys ; sys.path.insert(0,'')" >> "%bindir%../python37embedded/Lib/site-packages/sitecustomize.py"
)
%python% -m pip --quiet install importlib.metadata
%python% -m pip --quiet uninstall --yes cwipc_util cwipc_codec cwipc_realsense2 cwipc_codec
%python% -m pip --quiet install --upgrade --find-links="%bindir%..\cwipc\python" cwipc_util cwipc_codec cwipc_realsense2 cwipc_codec
pushd %bindir%..
if exist "%bindir%..\python37embedded\Scripts\" (
    xcopy /y "%bindir%..\python37embedded\Scripts\cwipc_*" "%bindir%..\bin"
)
popd