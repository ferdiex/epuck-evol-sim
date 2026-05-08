@echo off
REM Cambiar al directorio del simulador
cd /d "C:\Users\fmont\easim"

REM Activar entorno epuck
call conda activate epuck

REM Ejecutar simulador
python view_epuck_sim.py

REM Mantener ventana abierta
pause
