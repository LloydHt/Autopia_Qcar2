# 🚗 Autopia QCar2 - Autonomous Navigation

Ce dépôt contient le workspace ROS 2 (`qcar2_ws`) dédié à la navigation autonome du robot QCar. 
Le projet implémente une architecture modulaire en C++ fusionnant la vision artificielle (suivi de ligne), la cartographie globale (HDMap) et la sécurité (LiDAR).

## 🧠 Architecture du Système

Le cœur du système repose sur un arbitrage dynamique des commandes de vitesse (`cmd_vel`), séparant la génération de trajectoires de la prise de décision.

### 1. Nœud d'Arbitrage (`cmd_vel_arbiter`)
C'est le "cerveau" décisionnel du robot. Il multiplexe les différentes sources de navigation selon une hiérarchie stricte et intègre un système de *Watchdog* (timeout) pour sécuriser le robot en cas de défaillance d'un capteur.

**Hiérarchie des priorités :**
1. 🚨 **Sécurité (LiDAR)** : Freinage d'urgence ou évitement d'obstacle immédiat.
2. 🛑 **Signalisation** : Arrêt aux panneaux STOP.
3. 🗺️ **Navigation Hybride (Caméra + Carte)** : 
   - La **vitesse** longitudinale (`linear.x`) est toujours dictée par la HDMap (respect des limitations).
   - La **direction** (`angular.z`) est dictée par la Caméra (suivi de ligne) pour la précision locale en temps normal.
   - *Forçage HDMap :* La carte prend le contrôle exclusif de la direction dans les **ronds-points**, lorsque la **ligne est perdue**, ou lors de **virages serrés** (intersections).
4. ⚠️ **Safety Stop** : Arrêt complet automatique si aucune source de navigation n'est active.

### 2. Nœud de Suivi de Ligne (`lane_following_node`)
- Traitement d'image via `cv_bridge` et OpenCV.
- Maintien du robot centré sur la ligne jaune (Contrôleur PID).
- **Dégradation Gracieuse (Fallback) :** Si la ligne est perdue, le nœud utilise l'inertie de l'erreur précédente pendant quelques frames, puis bascule sur une détection de l'herbe (considérée comme un mur virtuel vert) pour guider le robot à l'aveugle dans les virages.

### 3. Nœud de Navigation Globale (`hdmap_navigation_node`)
- Suit une trajectoire précalculée basée sur un fichier de mission (*waypoints*).
- Calcule l'erreur angulaire vers le prochain objectif.
- Détecte le contexte routier (ex: signale à l'arbitre que le robot entre dans un rond-point).

## 🚀 Installation & Compilation

Prérequis : Ubuntu 22.04 / 24.04 avec ROS 2 installé.
```bash
# Cloner le dépôt
cd ~/qcar2_ws/src
git clone [https://github.com/LloydHt/Autopia_Qcar2.git](https://github.com/LloydHt/Autopia_Qcar2.git) .

# Compiler le projet
cd ~/qcar2_ws
colcon build
source install/setup.bash
