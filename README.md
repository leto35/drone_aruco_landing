# Drone Autonome - Atterrissage sur Cible Mobile ArUco

Projet de développement d'un système de drone autonome capable de détecter et d'atterrir sur une cible mobile équipée d'un marqueur ArUco.

## Description du Projet

Ce projet implémente un système complet de contrôle de drone permettant :
- Le décollage automatique et la montée à une altitude de travail
- La détection en temps réel de marqueurs ArUco via la caméra embarquée
- Le suivi précis d'une cible en mouvement
- L'atterrissage autonome sur la cible mobile

Le système utilise un contrôleur PID adaptatif qui ajuste ses paramètres en fonction de l'altitude pour garantir une précision optimale lors de la descente.

## Technologies Utilisées

- **Simulation** : Gazebo Harmonic (gz-sim)
- **Autopilote** : PX4-Autopilot
- **Communication drone** : MAVSDK (C++)
- **Vision par ordinateur** : OpenCV 4.5.4 avec module ArUco
- **Langage** : C++17
- **Build system** : CMake

## Prérequis

### Logiciels requis
- Ubuntu 22.04 (ou WSL2 avec Ubuntu 22.04)
- Gazebo Harmonic 8.x
- PX4-Autopilot
- OpenCV 4.5.4+
- MAVSDK 3.x
- CMake 3.22+
- GCC/G++ avec support C++17

### Installation des dépendances principales

```bash
# Gazebo Harmonic
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt install gz-harmonic -y

# OpenCV
sudo apt install libopencv-dev libopencv-contrib-dev -y

# MAVSDK
wget https://github.com/mavlink/MAVSDK/releases/download/v3.10.1/libmavsdk-dev_3.10.1_ubuntu22.04_amd64.deb
sudo dpkg -i libmavsdk-dev_3.10.1_ubuntu22.04_amd64.deb
```

## Installation

### 1. Cloner le projet
```bash
git clone https://github.com/leto35/drone_aruco_landing.git
cd drone_aruco_landing
```

### 2. Installer PX4-Autopilot
```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git submodule update --init --recursive
bash ./Tools/setup/ubuntu.sh
make px4_sitl gz_x500_gimbal
```

### 3. Installer le modèle du robot ArUco
```bash
# Copier les fichiers du modèle
cp ~/drone_aruco_landing/model.sdf ~/PX4-Autopilot/Tools/simulation/gz/models/aruco_robot/
cp ~/drone_aruco_landing/default.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
```

### 4. Compiler le projet
```bash
cd ~/drone_aruco_landing
mkdir -p build && cd build
cmake ..
make
```

## Utilisation

### Lancement de la simulation complète

**Terminal 1 : Démarrer PX4 et Gazebo**
```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500_gimbal
```
Attendre environ 30 secondes que Gazebo s'ouvre complètement.

**Terminal 2 : Spawner le robot ArUco**
```bash
gz service -s /world/default/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req "sdf_filename: '$HOME/PX4-Autopilot/Tools/simulation/gz/models/aruco_robot/model.sdf', name: 'aruco_robot', pose: {position: {x: 5, y: 0, z: 0}}"
```

**Terminal 3 : Lancer le programme de contrôle**
```bash
cd ~/drone_aruco_landing/build
./my_drone_pid
```
Attendre que le drone décolle et atteigne 10m d'altitude.

**Terminal 2 : Faire bouger le robot (après décollage)**
```bash
gz topic -t /cmd_vel -m gz.msgs.Twist -p "linear: {x: -0.8}"
```

### Déroulement de la mission

1. Le drone décolle automatiquement et monte à 10m
2. La caméra s'oriente vers le bas
3. Le robot ArUco commence à se déplacer
4. Le drone détecte le marqueur ArUco et commence le suivi
5. Le contrôleur PID ajuste la position du drone pour rester centré sur la cible
6. Le drone descend progressivement en maintenant le centrage
7. L'atterrissage se déclenche automatiquement en dessous de 15cm

## Architecture du Code

### Classes principales

- **CameraHandler** : Gestion du flux vidéo depuis Gazebo
- **MarkerDetector** : Détection des marqueurs ArUco dans les images
- **AdaptiveController** : Implémentation du contrôleur PID avec gains adaptatifs
- **FlightManager** : Gestion complète du vol (décollage, offboard, atterrissage)

### Paramètres PID

Le système utilise deux contrôleurs PID (latéral et longitudinal) avec des gains qui s'adaptent selon l'altitude :

- **Haute altitude (>2m)** : Gains élevés pour réactivité (Kp=2.0, Ki=0.5, Kd=0.3)
- **Basse altitude (<2m)** : Gains réduits pour stabilité (Kp=1.5, Ki=0.3, Kd=0.2)

### Vitesses de descente adaptatives

- Altitude > 3m : 0.4 m/s
- 1.5m < Altitude < 3m : 0.3 m/s
- Altitude < 1.5m : 0.2 m/s

## Fonctionnalités

- ✅ Décollage automatique avec montée à altitude définie
- ✅ Détection robuste de marqueurs ArUco 4x4 (dictionnaire 50)
- ✅ Suivi temps réel de cible mobile
- ✅ Contrôle PID adaptatif selon l'altitude
- ✅ Gestion de perte temporaire de cible (descente lente en recherche)
- ✅ Atterrissage précis automatique
- ✅ Interface visuelle (fenêtre OpenCV avec visualisation du tracking)

## Personnalisation

### Modifier l'altitude de travail
Dans `my_drone_pid.cpp`, fonction `execute_takeoff()`, ligne ~90 :
```cpp
10.0f,  // Modifier cette valeur (en mètres)
```

### Modifier les gains PID
Dans `my_drone_pid.cpp`, fonction `update_position()`, lignes ~220-225 :
```cpp
lateral_pid_->adjust_gains(2.0f, 0.5f, 0.3f);  // Kp, Ki, Kd
```

### Modifier la vitesse du robot
Lors du lancement du mouvement :
```bash
gz topic -t /cmd_vel -m gz.msgs.Twist -p "linear: {x: -0.8}"  # Changer -0.8
```

## Dépannage

### Le drone ne s'arme pas
- Vérifier que PX4 affiche "Ready for takeoff"
- Attendre 30-40 secondes après le lancement de Gazebo

### Pas de détection ArUco
- Vérifier que le robot est bien spawné dans Gazebo
- Vérifier que la caméra pointe vers le bas (gimbal à -90°)
- Augmenter l'altitude de départ si nécessaire

### Erreur de compilation
```bash
# Vérifier les versions
gz sim --version  # Doit être 8.x
pkg-config --modversion opencv4  # Doit être >= 4.5
```

## Licence

Ce projet est sous licence MIT - voir le fichier LICENSE pour plus de détails.

## Auteur

**Lucas** - Projet de système de drone autonome avec atterrissage précis sur cible mobile

---

*Développé dans le cadre d'un projet académique sur les systèmes autonomes et la vision par ordinateur.*
