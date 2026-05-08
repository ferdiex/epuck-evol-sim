#!/bin/zsh
# Log para verificar entorno y fecha
echo "$(date): Usando Python en: $(which python)" >> ~/epuck_log.txt

# Activa el entorno Conda
source /Users/fmontes/miniforge3/bin/activate epuck

# Cambia al directorio del proyecto
cd "/Users/fmontes/GitHub/Epuck/public"

# Ejecuta el script principal
python3 view_epuck_sim.py
