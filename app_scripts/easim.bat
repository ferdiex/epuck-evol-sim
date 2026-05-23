@echo off
REM Change to simulator directory
cd /d "C:\Users\user_name\easim"

REM Activate epuck environment
call conda activate epuck

REM Run simulator
python view_epuck_sim.py

REM Keep window open
pause
