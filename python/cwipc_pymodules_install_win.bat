@echo on
set "params=%*"
cd /d "%~dp0" && ( if exist "%temp%\getadmin.vbs" del "%temp%\getadmin.vbs" ) && fsutil dirty query %systemdrive% 1>nul 2>nul || (  echo Set UAC = CreateObject^("Shell.Application"^) : UAC.ShellExecute "cmd.exe", "/k cd ""%~sdp0"" && %~s0 %params%", "", "runas", 1 >> "%temp%\getadmin.vbs" && "%temp%\getadmin.vbs" && exit /B )

set bindir=%~dp0
set python=python
if exist %bindir%..\python37embedded\python.exe (
	set python="%bindir%..\python37embedded\python.exe"
)
pushd %bindir%..\share\cwipc_util\python
%python% setup.py install
if exist %bindir%..\share\cwipc_realsense2\python\setup.py (
	pushd %bindir%..\share\cwipc_realsense2\python
	%python% setup.py install
)
if exist %bindir%..\share\cwipc_kinect\python\setup.py (
	pushd %bindir%..\share\cwipc_kinect\python
	%python% setup.py install
)
pushd %bindir%..\share\cwipc_codec\python
%python% setup.py install

echo "You should also manually copy cwipc scripts from python37embedded/Scripts into bin directory"
exit