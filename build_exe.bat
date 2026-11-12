@echo off
REM Builds the portable single-file executable: dist\CoilWinder.exe
REM The precompiled firmwares (firmware\*.hex) and the sketches are embedded in the exe,
REM so it can flash the Arduino without arduino-cli / the Arduino IDE.
cd /d "%~dp0"

if exist .venv\Scripts\python.exe (set PY=.venv\Scripts\python.exe) else (set PY=python)

%PY% -m pip install --upgrade pyserial pyinstaller || goto :error

%PY% -m PyInstaller --noconfirm --clean --onefile --windowed --name CoilWinder ^
    --add-data "firmware;firmware" ^
    --add-data "bobin_v0;bobin_v0" ^
    --add-data "current;current" ^
    main.py || goto :error

echo.
echo Build OK: dist\CoilWinder.exe
exit /b 0

:error
echo.
echo Build FAILED
exit /b 1
