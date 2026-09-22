@echo off
title Build Run_Simulation.exe
cd /d "%~dp0"

echo [*] Generating standalone single-file HTML bundle...
python bundle_standalone.py

echo [*] Compiling Run_Simulation.cs with embedded resources using native Windows .NET compiler...
C:\Windows\Microsoft.NET\Framework64\v4.0.30319\csc.exe /out:Run_Simulation.exe /target:exe /optimize+ /resource:PGK_Simulation_Standalone.html Run_Simulation.cs

if %ERRORLEVEL% equ 0 (
    echo.
    echo [SUCCESS] Run_Simulation.exe built successfully!
) else (
    echo.
    echo [ERROR] Compilation failed.
)
pause
