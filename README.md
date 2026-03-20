
# NavelTrajectory

Trajectory-Planning-System für soziale Roboter auf Basis von ROS 2 Jazzy, Webots und dem Nav2-Stack.
Das System implementiert eine RRT*-basierte Globalplanung mit kubischer B-Spline-Glättung und
einem MPPI-Controller, verglichen mit einer NavFn/DWB-Baseline.

---

## Voraussetzungen

| Software | Version | Link |
|---|---|---|
| Ubuntu | 24.04 LTS | https://ubuntu.com/download |
| ROS 2 | Jazzy | https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html |
| Webots | aktuelle stabile Version | https://cyberbotics.com |
| webots_ros2_driver | Jazzy | https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html |

---

## Installation

### 1. ROS 2 Jazzy

Folge der offiziellen Anleitung. Installiere zusaetzlich die Dev-Tools fuer colcon:
```bash
sudo apt install ros-dev-tools
```

### 2. Webots und webots_ros2_driver

Folge der offiziellen Webots-ROS-2-Anleitung (Link in der Tabelle oben).

### 3. Nav2
```bash
sudo apt install ros-jazzy-nav2
```

### 4. CycloneDDS (empfohlene RMW-Implementierung)
```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
```

Füge folgende Zeile in `~/.bashrc` ein:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Anschließend:
```bash
source ~/.bashrc
```

### 5. Teleop Keyboard (optional, für manuelle Steuerung)
```bash
sudo apt install ros-jazzy-teleop-twist-keyboard
```

### 6. Shell-Skripte ausfuehrbar machen
```bash
cd webots_ros
chmod +x *.sh
```

---

## Coding Style

Alle Dateien folgen dem Google Style Guide:
https://google.github.io/styleguide/

---

## Verwendung

### Allgemeiner Ablauf
```bash
cd webots_ros
./start.sh           # Startet Webots + ROS 2 Basis-Stack
./navigation.sh      # Startet Nav2 (Standard-Config: nav2_params.yaml)
./test.sh <testfall> <name>
```

Der Parameter `<name>` wird als Label in den Evaluierungsergebnissen verwendet.

Um eine alternative Nav2-Konfiguration zu laden:
```bash
./navigation.sh <config_datei.yaml>
```

---

## Testfälle

### Test 1 -  Go-To-Pose

**Szenario:** Direktnavigation zu einem Zielpunkt mit initialer Rotation.
```bash
./start.sh
./navigation.sh <config_datei.yaml>
./test.sh test_1 "<name>"
```

---

### Test 2 - Besucherszenario mit Zwischenstopp

**Szenario:** Navigation zu einem Zwischenziel, dann zum Endziel. Der Roboter prüft
an jedem Checkpoint, ob der Besucher noch folgt (Bewegungsdetektion via Kamera).

Die Ergebnisse werden pro Wegpunkt separat gespeichert.
```bash
./start.sh
./navigation.sh <config_datei.yaml>
./test.sh test_2 "<name>"
```

**Wichtig:** Am Start und an jedem Checkpoint muss der simulierte Fussgänger sichtbar
in Bewegung sein. Steuerung des Fussgängers per Teleop:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=person_1/cmd_vel
```

Alternativ: Fussgänger manuell in der Webots-Szene verschieben (Drag & Drop).

---

### Test 3 - Dynamische Hindernisse

**Szenario:** Drei Roboter in der Szene. Mit erstem Roboter sollte Konfrontation geschehen.
```bash
./start_multiple.sh    # startet mehrere Roboter-Instanzen
./navigation.sh <config_datei.yaml>
./test.sh test_3 "<name>"
```
