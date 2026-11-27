@echo off
cd /d "%~dp0"
if exist ".venv\Scripts\python.exe" (
  ".venv\Scripts\python.exe" rx_viewer.py
) else (
  python rx_viewer.py
)
if errorlevel 1 (
  echo.
  echo Install dependencies: python -m pip install -r requirements.txt
  pause
)
