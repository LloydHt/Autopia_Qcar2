#pragma once

#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <algorithm>

// =========================================================
// HD MAP AUTOPIA — Ligne droite SUD→NORD
//
// Coordonnées Ground Truth Gazebo mesurées :
//   Spawn      : x=0.30, y=-8.00  yaw=90°
//   Entrée RP  : x=0.30, y=-1.69
//   Centre RP  : x=0.30, y= 0.035  r=1.725m
//   Sortie RP  : x=0.30, y=+1.76
//   Fin NORD   : x=0.30, y=+8.00
//
// Pas de STOP sur la ligne droite (décision projet)
// =========================================================

constexpr double WAYPOINT_RADIUS = 0.35;
constexpr double RP_CX           = 0.300;
constexpr double RP_CY           = 0.035;
constexpr double RP_R            = 1.725;

struct HDMapNode {
  std::string node_id;
  double x, y;
  bool   has_stop    = false;
  double stop_x      = 0.0, stop_y = 0.0;
  std::string node_type   = "normal";
  double speed_limit = 0.20;
  std::vector<std::string> next_nodes;

  double distanceTo(double rx, double ry) const {
    return std::sqrt(std::pow(x-rx,2) + std::pow(y-ry,2));
  }
  double distanceToStop(double rx, double ry) const {
    return std::sqrt(std::pow(stop_x-rx,2) + std::pow(stop_y-ry,2));
  }
};

using Mission = std::vector<std::string>;

class AutopiaHDMap {
public:
  AutopiaHDMap() { buildMap(); }

  bool loadMission(const std::string& name) {
    auto it = missions_.find(name);
    if (it == missions_.end()) return false;
    current_mission_ = it->second;
    current_index_   = 0;
    return true;
  }

  const HDMapNode* getCurrentWaypoint() const {
    if (current_index_ >= (int)current_mission_.size()) return nullptr;
    auto it = nodes_.find(current_mission_[current_index_]);
    return (it != nodes_.end()) ? &it->second : nullptr;
  }

  void advanceWaypoint() { current_index_++; }

  bool isMissionComplete() const {
    return current_index_ >= (int)current_mission_.size();
  }

  double getDistanceToCurrent(double rx, double ry) const {
    const auto* wp = getCurrentWaypoint();
    return wp ? wp->distanceTo(rx, ry) : 0.0;
  }

  double getHeadingToCurrent(double rx, double ry) const {
    const auto* wp = getCurrentWaypoint();
    return wp ? std::atan2(wp->y - ry, wp->x - rx) : 0.0;
  }

  bool isWaypointReached(double rx, double ry) const {
    const auto* wp = getCurrentWaypoint();
    if (!wp) return false;
    double r = (wp->node_type == "roundabout") ? WAYPOINT_RADIUS * 0.8
                                                : WAYPOINT_RADIUS;
    return wp->distanceTo(rx, ry) < r;
  }

  double getCurrentSpeedLimit() const {
    const auto* wp = getCurrentWaypoint();
    return wp ? wp->speed_limit : 0.20;
  }

  std::pair<double,bool> getDistanceToNextStop(double rx, double ry) const {
    int end = std::min(current_index_ + 5, (int)current_mission_.size());
    for (int i = current_index_; i < end; i++) {
      auto it = nodes_.find(current_mission_[i]);
      if (it != nodes_.end() && it->second.has_stop)
        return {it->second.distanceToStop(rx, ry), true};
    }
    return {999.0, false};
  }

  bool isInRoundabout() const {
    const auto* wp = getCurrentWaypoint();
    return wp && wp->node_type == "roundabout";
  }

  int getCurrentIndex() const { return current_index_; }
  int getMissionSize()  const { return (int)current_mission_.size(); }

private:
  std::map<std::string, HDMapNode> nodes_;
  std::map<std::string, Mission>   missions_;
  Mission current_mission_;
  int     current_index_ = 0;

  void addRP(const std::string& id, double angle_deg, double speed = 0.12) {
    double a = angle_deg * M_PI / 180.0;
    HDMapNode n;
    n.node_id     = id;
    n.x           = RP_CX + RP_R * std::cos(a);
    n.y           = RP_CY + RP_R * std::sin(a);
    n.node_type   = "roundabout";
    n.speed_limit = speed;
    nodes_[id]    = n;
  }

  void buildMap() {
    auto add = [&](HDMapNode n) { nodes_[n.node_id] = n; };

    // =======================================================
    // AXE SUD — Spawn → Entrée rond-point
    // x=0.30 fixe, y croissant, pas de STOP
    // =======================================================
    add({"SUD_SPAWN",       0.30, -8.00, false, 0, 0, "normal", 0.20});
    add({"SUD_S1",          0.30, -7.00, false, 0, 0, "normal", 0.20});
    add({"SUD_S2",          0.30, -6.00, false, 0, 0, "normal", 0.20});
    add({"SUD_S3",          0.30, -5.00, false, 0, 0, "normal", 0.20});
    add({"SUD_S4",          0.30, -4.00, false, 0, 0, "normal", 0.20});
    add({"SUD_S5",          0.30, -3.00, false, 0, 0, "normal", 0.20});
    add({"SUD_S6",          0.30, -2.20, false, 0, 0, "normal", 0.15});
    add({"SUD_RP_APPROCHE", 0.30, -1.90, false, 0, 0, "normal", 0.12});

    // =======================================================
    // ROND-POINT — Centre (0.30, 0.035), r=1.725m
    // Anti-horaire -90° → +90° (SUD → NORD, 2ème sortie)
    //
    //  RP_N90 : ( 0.300, -1.690)  ← entrée SUD
    //  RP_P00 : ( 2.025, +0.035)  ← point EST (max droite)
    //  RP_P90 : ( 0.300, +1.760)  ← sortie NORD
    // =======================================================
    addRP("RP_N90",  -90, 0.12);
    addRP("RP_N75",  -75, 0.12);
    addRP("RP_N60",  -60, 0.12);
    addRP("RP_N45",  -45, 0.12);
    addRP("RP_N30",  -30, 0.12);
    addRP("RP_N15",  -15, 0.12);
    addRP("RP_P00",    0, 0.12);
    addRP("RP_P15",   15, 0.12);
    addRP("RP_P30",   30, 0.12);
    addRP("RP_P45",   45, 0.12);
    addRP("RP_P60",   60, 0.12);
    addRP("RP_P75",   75, 0.12);
    addRP("RP_P90",   90, 0.12);

    // =======================================================
    // AXE NORD — Sortie rond-point → Fin
    // x=0.30 fixe, y croissant, pas de STOP
    // =======================================================
    add({"NORD_RP_SORTIE", 0.30,  1.90, false, 0, 0, "normal", 0.15});
    add({"NORD_S1",        0.30,  2.50, false, 0, 0, "normal", 0.18});
    add({"NORD_S2",        0.30,  3.50, false, 0, 0, "normal", 0.20});
    add({"NORD_S3",        0.30,  4.50, false, 0, 0, "normal", 0.20});
    add({"NORD_S4",        0.30,  5.50, false, 0, 0, "normal", 0.20});
    add({"NORD_S5",        0.30,  6.50, false, 0, 0, "normal", 0.20});
    add({"NORD_S6",        0.30,  7.00, false, 0, 0, "normal", 0.20});
    add({"NORD_FIN",       0.30,  8.00, false, 0, 0, "normal", 0.20});

    // =======================================================
    // MISSION — Ligne droite SUD → NORD
    // =======================================================
    missions_["test_ligne_droite"] = {
      // Axe SUD
      "SUD_SPAWN",
      "SUD_S1", "SUD_S2", "SUD_S3", "SUD_S4",
      "SUD_S5", "SUD_S6", "SUD_RP_APPROCHE",
      // Rond-point
      "RP_N90", "RP_N75", "RP_N60", "RP_N45",
      "RP_N30", "RP_N15", "RP_P00", "RP_P15",
      "RP_P30", "RP_P45", "RP_P60", "RP_P75", "RP_P90",
      // Axe NORD
      "NORD_RP_SORTIE",
      "NORD_S1", "NORD_S2", "NORD_S3",
      "NORD_S4", "NORD_S5", "NORD_S6",
      "NORD_FIN"
    };
  }
};