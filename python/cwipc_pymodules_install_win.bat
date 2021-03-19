@echo on
set "params=%*"
cd /d "%~dp0" && ( if exist "%temp%\getadmin.vbs" del "%temp%\getadmin.vbs" ) && fsutil dirty query %systemdrive% 1>nul 2>nul || (  echo Set UAC = CreateObject^("Shell.Application"^) : UAC.ShellExecute "cmd.exe", "/k cd ""%~sdp0"" && %~s0 %params%", "", "runas", 1 >> "%temp%\getadmin.vbs" && "%temp%\getadmin.vbs" && exit /B )

set bindir=%~dp0
set python=python
if exist %bindir%..\python37embedded\python.exe (
	set python="%bindir%..\python37embedded\python.exe"
)
%python% -m pip install importlib.metadata
pushd %bindir%..\share\cwipc_util\python
%python% setup.py install
popd
if exist %bindir%..\share\cwipc_realsense2\python\setup.py (
	pushd %bindir%..\share\cwipc_realsense2\python
	%python% setup.py install
	popd
)
if exist %bindir%..\share\cwipc_kinect\python\setup.py (
	pushd %bindir%..\share\cwipc_kinect\python
	%python% setup.py install
	popd
)
pushd %bindir%..\share\cwipc_codec\python
%python% setup.py install
popd
pushd %bindir%..
if exist %bindir%..\python37embedded\Scripts\ (
	xcopy /y "%bindir%..\python37embedded\Scripts\cwipc_*" "%bindir%..\bin"
)
popd