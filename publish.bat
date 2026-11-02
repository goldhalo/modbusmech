@echo off
rem ==========================================================================
rem  publish.bat - build modbusmech (+ netmech) static libs, publish to 3rdlibs
rem
rem    publish.bat          Release  (/MD)
rem    publish.bat debug    Debug    (/MDd)  -- moc / EAS use this one
rem
rem  Output
rem    D:\3rdlibsIes\3rdlibs\include\modbusmech\*.hpp
rem    D:\3rdlibsIes\3rdlibs\include\netmech\*.hpp
rem    D:\3rdlibsIes\3rdlibs\lib\modbusmech[ D].lib
rem    D:\3rdlibsIes\3rdlibs\lib\NetMechanism[ D].lib
rem
rem  Notes
rem    - netmech is taken from thirdparty/netmech (bundled copy).
rem      E:\git\netmech is the newer "no serial" architecture and is NOT
rem      compatible with this library (it has no netmech/serial.hpp).
rem    - CRT is /MD or /MDd only, never /MT (netmech CMake enforces the same).
rem    - Keep CRT identical to the consumer: moc / EAS are /MDd, so pass debug.
rem    - ASCII only, no BOM: cmd parses this file as GBK and would drop chars.
rem ==========================================================================
setlocal enabledelayedexpansion

set "CFG=%~1"
if "%CFG%"=="" set "CFG=release"

pushd "%~dp0"
set "ROOT=%CD%"
popd

rem ---- toolchain (same set as ipms .review scripts) ----
set "VCV=C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Tools\MSVC\14.28.29333"
set "SDK=C:\Program Files (x86)\Windows Kits\10"
set "SDKI=%SDK%\Include\10.0.18362.0"
set "SDKL=%SDK%\Lib\10.0.18362.0"
set "PATH=%VCV%\bin\Hostx64\x64;%SDK%\bin\10.0.18362.0\x64;%PATH%"
set "INCLUDE=%VCV%\include;%SDKI%\ucrt;%SDKI%\um;%SDKI%\shared;%SDKI%\winrt"
set "LIB=%VCV%\lib\x64;%SDKL%\ucrt\x64;%SDKL%\um\x64"

rem ---- config ----
if /I "%CFG%"=="debug" (
    set "CRT=/MDd"
    set "SUF=D"
    set "OPT=/Od /Zi"
) else (
    set "CRT=/MD"
    set "SUF="
    set "OPT=/O2"
)

set "NMR=%ROOT%\thirdparty\netmech"
set "ASIO=%NMR%\thirdparty\asio\include"
set "NMI=%NMR%\include"
set "NMS=%NMR%\src"
set "MMI=%ROOT%\include"
set "MMS=%ROOT%\src"

set "OBJ=%ROOT%\build\publish_%CFG%"
if exist "%OBJ%" rd /s /q "%OBJ%"
mkdir "%OBJ%\netmech" 2>nul
mkdir "%OBJ%\modbusmech" 2>nul

set "DEFS=/D_WIN32_WINNT=0x0601 /DWIN32_LEAN_AND_MEAN /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS"
set "INCS=/I"%ASIO%" /I"%NMI%" /I"%NMS%" /I"%MMI%" /I"%MMS%""
set "OPTS=/nologo /c /std:c++17 /EHsc %CRT% %OPT% /Fd"%OBJ%\vc140.pdb" /W4 /bigobj /permissive- /Zc:__cplusplus /utf-8 /wd4819"

echo ===== [1/4] netmech =====
for %%f in ("%NMS%\*.cpp") do (
    echo   %%~nxf
    cl %OPTS% %DEFS% %INCS% /Fo"%OBJ%\netmech\\" /Tp"%%~f" || goto :err
)

echo ===== [2/4] modbusmech =====
for %%f in ("%MMS%\*.cpp") do (
    echo   %%~nxf
    cl %OPTS% %DEFS% %INCS% /Fo"%OBJ%\modbusmech\\" /Tp"%%~f" || goto :err
)

echo ===== [3/4] lib =====
lib /nologo /OUT:"%OBJ%\NetMechanism%SUF%.lib" "%OBJ%\netmech\*.obj" || goto :err
lib /nologo /OUT:"%OBJ%\modbusmech%SUF%.lib" "%OBJ%\modbusmech\*.obj" || goto :err

echo ===== [4/4] publish =====
set "DST=D:\3rdlibsIes\3rdlibs"
if not exist "%DST%\include\modbusmech" mkdir "%DST%\include\modbusmech"
if not exist "%DST%\include\netmech" mkdir "%DST%\include\netmech"
copy /Y "%MMI%\modbusmech\*.hpp" "%DST%\include\modbusmech\" || goto :err
copy /Y "%NMI%\netmech\*.hpp" "%DST%\include\netmech\" || goto :err
copy /Y "%OBJ%\modbusmech%SUF%.lib" "%DST%\lib\" || goto :err
copy /Y "%OBJ%\NetMechanism%SUF%.lib" "%DST%\lib\" || goto :err

echo.
echo DONE. cfg=%CFG%  (crt=%CRT%)
echo   %DST%\include\modbusmech
echo   %DST%\lib\modbusmech%SUF%.lib
echo   %DST%\lib\NetMechanism%SUF%.lib
goto :eof

:err
echo.
echo FAILED. rc=%ERRORLEVEL%
exit /b 1
