#!/bin/bash


# Récupérer le répertoire où se trouve le script (sous forme WSL)
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Convertir le chemin du fichier config.xlaunch en chemin Windows
CONFIG_XLAUNCH_WIN_PATH=$(wslpath -w "$SCRIPT_DIR/config.xlaunch")

# Vérifier si le fichier config.xlaunch existe
if [ -f "$SCRIPT_DIR/config.xlaunch" ]; then
    echo "Launching VcXsrv with config.xlaunch..."
    # Exécuter le fichier .xlaunch avec vcxsrv (chemin Windows)
    cmd.exe /c "$CONFIG_XLAUNCH_WIN_PATH"
else
    echo "Error: config.xlaunch not found in the script's directory."
    exit 1
fi

# Exporter la variable DISPLAY
export DISPLAY=:0
echo "DISPLAY set to :0"

# Message indiquant que le script est terminé
echo "X server launched and DISPLAY is set."
