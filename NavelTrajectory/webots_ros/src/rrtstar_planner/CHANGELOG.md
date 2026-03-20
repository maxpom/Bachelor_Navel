# Changelog - RRT* Planner Adaptierung

## Basis-Implementation
**Quelle:** mmcza/TurtleBot-RRT-Star  
**Repository:** https://github.com/mmcza/TurtleBot-RRT-Star  
**Lizenz:** Apache-2.0  
**Original-Version:** Commit d820402 (2024)  
**Angepasst für:** Navel Trajectory Navigation System  
**Autor der Anpassungen:** Maximilian Pomplun  
**Datum:** 04.03.2026

---

## Änderungen für ROS2 Jazzy Kompatibilität

### API-Anpassungen
- **createPlan() Signatur erweitert**
  - **Grund:** ROS2 Jazzy API-Änderung in nav2_core::GlobalPlanner
  - **Änderung:** Hinzufügen des `std::function<bool()> cancel_checker` Parameters
  - **Dateien:** 
    - `include/rrtstar_planner/rrtstar_planner.hpp` (Zeile 42-44)
    - `src/rrtstar_planner.cpp` (Zeile 158-161)

### Funktionale Erweiterungen
- **Cancellation-Support implementiert**
  - **Grund:** Ermöglicht Abbruch laufender Planungen durch Nav2
  - **Änderung:** Einfügen von Cancellation-Checks in Hauptschleife
  - **Datei:** `src/rrtstar_planner.cpp` (Zeile ~213-217)
  - **Code:**
```cpp
    if (cancel_checker && cancel_checker()) {
        RCLCPP_INFO(node_->get_logger(), "RRT* planning was cancelled");
        return global_path;
    }
```

