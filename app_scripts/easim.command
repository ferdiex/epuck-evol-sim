#!/bin/zsh
# Log to verify environment and date
echo "$(date): Usando Python en: $(which python)" >> ~/epuck_log.txt

# Activate Conda environment
source /Users/user_name/miniforge3/bin/activate epuck

# Change to project directory
cd "/Users/user_name/GitHub/Epuck/public"

# Run main script
python3 view_epuck_sim.py
