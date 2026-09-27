@echo off
if not exist build mkdir build
call "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 >nul
cl /nologo /O2 /EHsc /std:c++17 /I"src\reserve_odometry\include" src\reserve_odometry\src\pipeline_cli.cpp /Fe:build\pipeline_cli.exe /Fo:build\pipeline_cli.obj
if errorlevel 1 exit /b 1
cl /nologo /O2 /EHsc /std:c++17 /I"src\reserve_odometry\include" src\reserve_odometry\src\core_cli.cpp /Fe:build\core_cli.exe /Fo:build\core_cli.obj
if errorlevel 1 exit /b 1
cl /nologo /O2 /EHsc /std:c++17 /I"src\reserve_odometry\include" src\reserve_odometry\src\route_project_cli.cpp /Fe:build\route_project_cli.exe /Fo:build\route_project_cli.obj
if errorlevel 1 exit /b 1
cl /nologo /O2 /EHsc /std:c++17 /I"src\reserve_odometry\include" tests\native_geometry.cpp /Fe:build\native_geometry.exe /Fo:build\native_geometry.obj
if errorlevel 1 exit /b 1
build\native_geometry.exe
