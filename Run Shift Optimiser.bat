@echo off
REM ============================================================
REM  Two-Speed Shift Optimiser - double-click launcher
REM  Keeps the window open if something goes wrong, so you can
REM  read the error instead of watching it flash past.
REM ============================================================
title Two-Speed Shift Optimiser
cd /d "%~dp0"

echo Starting Two-Speed Shift Optimiser...
echo Folder: %CD%
echo.

python shift_app.py

if errorlevel 1 (
    echo.
    echo ============================================================
    echo  The app exited with an error - the message is above.
    echo.
    echo  Common fixes:
    echo    * "python is not recognized"
    echo         Python is not on PATH. Reinstall Python and tick
    echo         "Add python.exe to PATH", or use the full path:
    echo         "%LOCALAPPDATA%\Programs\Python\Python310\python.exe" shift_app.py
    echo.
    echo    * "No module named customtkinter"  ^(or matplotlib, scipy, ...^)
    echo         Run:  python -m pip install customtkinter matplotlib numpy pandas scipy openpyxl
    echo ============================================================
    echo.
    pause
)
