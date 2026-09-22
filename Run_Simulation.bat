@echo off
title 155mm PGK Digital Twin Simulation Launcher
cd /d "%~dp0"

echo ===============================================================================
echo   155MM PRECISION GUIDANCE KIT (PGK) & SMART FUZE DIGITAL TWIN
echo   SIH 2026 | Problem Statement ID: SIH26098 | Yantra India Limited
echo ===============================================================================
echo.

if exist "Run_Simulation.exe" (
    start "" "Run_Simulation.exe"
    exit /b
)

echo [*] Starting Python HTTP server on port 8080...
start "" python -m http.server 8080
timeout /t 2 >nul
echo [*] Opening simulation in your default browser...
start http://localhost:8080/
echo.
echo Server running at http://localhost:8080/
echo Close this window to keep server running in background.
pause
