@echo off
:loop
wchisp-win-x64\wchisp flash ..\..\Sources\2024_blaster\obj\Blaster2024.hex
timeout /t 2 /nobreak
goto loop
