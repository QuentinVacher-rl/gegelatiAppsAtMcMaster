#!/bin/bash

# Mettre à jour et installer wget et unzip si nécessaire
sudo apt update
sudo apt install -y wget unzip

# Créer le répertoire pour le projet
LIB_DIR=$(pwd)

# Version de MuJoCo
MUJOCO_VERSION=210

# Télécharger MuJoCo
echo "Téléchargement de MuJoCo..."
MUJOCO_URL="https://mujoco.org/download/mujoco${MUJOCO_VERSION}-linux-x86_64.tar.gz"
wget -O mujoco${MUJOCO_VERSION}-linux-x86_64.tar.gz "$MUJOCO_URL"

# Vérifier si le téléchargement a réussi
if [ $? -ne 0 ]; then
    echo "Erreur lors du téléchargement de MuJoCo."
    exit 1
fi

# Décompresser le fichier téléchargé
echo "Décompression de MuJoCo..."
tar -xzf mujoco${MUJOCO_VERSION}-linux-x86_64.tar.gz

# Vérifier si la décompression a réussi
if [ $? -ne 0 ]; then
    echo "Erreur lors de la décompression de MuJoCo."
    rm mujoco${MUJOCO_VERSION}-linux-x86_64.tar.gz
    exit 1
fi

# Supprimer le fichier archive
echo "Nettoyage des fichiers temporaires..."
rm mujoco${MUJOCO_VERSION}-linux-x86_64.tar.gz

# Déplacer MuJoCo dans le répertoire lib
echo "Déplacement de MuJoCo dans le répertoire lib..."
if [ ! -d "$LIB_DIR/mujoco${MUJOCO_VERSION}" ]; then
    mkdir -p "$LIB_DIR/mujoco${MUJOCO_VERSION}"
fi
mv mujoco210 "mujoco"

# Vérifier que MuJoCo a été installé correctement
echo "Vérification de l'installation de MuJoCo..."
if [ -d "$LIB_DIR/mujoco" ]; then
    echo "MuJoCo a été installé avec succès dans $LIB_DIR/mujoco/."
else
    echo "Erreur : le répertoire de MuJoCo n'existe pas."
    exit 1
fi

echo "Vous pouvez maintenant configurer votre projet CMake."
