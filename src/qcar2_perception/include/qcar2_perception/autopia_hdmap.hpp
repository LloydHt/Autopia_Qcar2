#pragma once

#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <algorithm>

// =========================================================
// HD MAP AUTOPIA — 2 missions
//
// Mission 1 : Ligne droite SUD→NORD via rond-point
// Mission 2 : Tour partiel avec vrais yaw mesurés
//
// Coordonnées Ground Truth Gazebo
// Robot toujours au milieu de sa voie
// =========================================================

constexpr double WAYPOINT_RADIUS = 0.35;

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
    double r = (wp->node_type == "roundabout") ? 0.25 : WAYPOINT_RADIUS;
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

  void buildMap() {
    auto add = [&](HDMapNode n) { nodes_[n.node_id] = n; };

    // =======================================================
    // MISSION 1 — Ligne droite SUD→NORD
    // x=0.30 fixe
    // =======================================================
    add({"SUD_SPAWN",        0.300, -8.000, false, 0, 0, "normal", 0.20});
    add({"SUD_S1",           0.300, -7.000, false, 0, 0, "normal", 0.20});
    add({"SUD_S2",           0.300, -6.000, false, 0, 0, "normal", 0.20});
    add({"SUD_S3",           0.300, -5.000, false, 0, 0, "normal", 0.20});
    add({"SUD_S4",           0.300, -4.000, false, 0, 0, "normal", 0.20});
    add({"SUD_S5",           0.300, -3.000, false, 0, 0, "normal", 0.20});
    add({"SUD_S6",           0.300, -2.200, false, 0, 0, "normal", 0.15});
    add({"SUD_RP_APPROCHE",  0.300, -1.800, false, 0, 0, "normal", 0.12});

    // Rond-point mission 1 — waypoints mesurés teleop
    add({"RP_P1", 0.300, -1.640, false, 0, 0, "roundabout", 0.12});
    add({"RP_P2", 0.540, -1.257, false, 0, 0, "roundabout", 0.12});
    add({"RP_P3", 0.696, -1.128, false, 0, 0, "roundabout", 0.12});
    add({"RP_P4", 0.941, -0.879, false, 0, 0, "roundabout", 0.12});
    add({"RP_P5", 1.190, -0.296, false, 0, 0, "roundabout", 0.12});
    add({"RP_P6", 1.060,  0.390, false, 0, 0, "roundabout", 0.12});
    add({"RP_P7", 0.700,  0.976, false, 0, 0, "roundabout", 0.12});
    add({"RP_P8", 0.540,  1.400, false, 0, 0, "roundabout", 0.12});
    add({"RP_P9", 0.300,  2.000, false, 0, 0, "roundabout", 0.12});

    add({"NORD_RP_SORTIE",   0.300,  2.200, false, 0, 0, "normal", 0.15});
    add({"NORD_S1",          0.300,  3.000, false, 0, 0, "normal", 0.18});
    add({"NORD_S2",          0.300,  4.000, false, 0, 0, "normal", 0.20});
    add({"NORD_S3",          0.300,  5.000, false, 0, 0, "normal", 0.20});
    add({"NORD_S4",          0.300,  6.000, false, 0, 0, "normal", 0.20});
    add({"NORD_S5",          0.300,  7.000, false, 0, 0, "normal", 0.20});
    add({"NORD_FIN",         0.300,  8.000, false, 0, 0, "normal", 0.20});

    // =======================================================
    // MISSION 2 — Tour partiel
    //
    // Voies respectées (milieu de voie) :
    //   NORD  → x=+0.300 (spawn→P2) puis x=+5.330 (P5→P6) puis x=-4.562 (P14→P15)
    //   EST   → y=-5.397 (P3→P4)   puis y=+4.609 (P15→P16)
    //   OUEST → y=+0.350 (P7→P13)
    //   FIN   → x=+0.300 (P17→P18)
    // =======================================================

    // ── SUD → Virage EST (P1→P3)
    // Monte NORD x=0.3, puis vire à droite vers EST
    add({"M2_P1",    0.300, -8.000, false, 0, 0, "normal", 0.20});
    add({"M2_P1a",    0.300, -7.500, false, 0, 0, "normal", 0.20});
    add({"M2_P1b",   0.300, -6.900, false, 0, 0, "normal", 0.20});
    add({"M2_P2",    0.300, -5.800, false, 0, 0, "normal", 0.18});
    add({"M2_P3",    0.614, -5.396, false, 0, 0, "normal", 0.15});

    // ── Ligne EST (P3→P4) — y=-5.397 fixe
    add({"M2_E1",    1.500, -5.397, false, 0, 0, "normal", 0.20});
    add({"M2_E2",    2.300, -5.397, false, 0, 0, "normal", 0.20});
    add({"M2_E3",    3.100, -5.397, false, 0, 0, "normal", 0.20});
    add({"M2_E4",    3.900, -5.397, false, 0, 0, "normal", 0.20});
    add({"M2_P4",    4.643, -5.398, false, 0, 0, "normal", 0.18});

    // ── Virage NORD (P4→P5) + montée NORD x=5.330
    add({"M2_P5",    5.233, -4.243, false, 0, 0, "normal", 0.15});
    add({"M2_N1",    5.330, -3.300, false, 0, 0, "normal", 0.18});
    add({"M2_N2",    5.330, -2.400, false, 0, 0, "normal", 0.18});
    add({"M2_P6",    5.330, -1.568, true, 5.330, -1.568, "normal", 0.15});

    // ── Virage OUEST (P6→P7) + ligne OUEST y=0.350
    // On commence à tourner plus tôt (x diminue progressivement) et on va vers y=+0.350
    add({"M2_W0",    5.050, -0.150, false, 0, 0, "normal", 0.15}); // Début de la courbe
    add({"M2_P7",    4.500,  0.150, false, 0, 0, "normal", 0.15}); // Milieu de la courbe (positif !)
    add({"M2_W1",    4.000,  0.350, false, 0, 0, "normal", 0.18}); // Fin de la courbe, aligné
    add({"M2_W2",    3.000,  0.350, false, 0, 0, "normal", 0.18});
    add({"M2_P8",    2.000,  0.350, false, 0, 0, "normal", 0.18});
    add({"M2_P9",    1.718,  0.342, false, 0, 0, "normal", 0.18});
    
    // ── Rond-point mission 2 — points mesurés avec teleop
    // Entrée EST → tour → sortie OUEST
    add({"M2_RP_E",    1.663,  0.373, false, 0, 0, "roundabout", 0.12}); // entrée
    add({"M2_RP_A1",   1.052,  0.971, false, 0, 0, "roundabout", 0.12});
    add({"M2_RP_A2",   0.051,  1.212, false, 0, 0, "roundabout", 0.12});
    add({"M2_RP_A3",   -0.686,  0.969, false, 0, 0, "roundabout", 0.12});
    add({"M2_RP_A4",  -1.332,  0.576, false, 0, 0, "roundabout", 0.12});
    add({"M2_RP_A5",  -1.847,  0.373, false, 0, 0, "roundabout", 0.12}); // sortie OUEST

    // ── Sortie RP EST → continue OUEST x décroissant y=0.365
    // P13 va OUEST, tourne droite au carrefour x=-4.562
    add({"M2_R1",    2.500,  0.365, false, 0, 0, "normal", 0.18});
    add({"M2_P13",   3.321,  0.365, false, 0, 0, "normal", 0.15});

    // ── Ligne OUEST (P13→virage) — y=0.365 fixe
    add({"M2_O1",    2.500,  0.365, false, 0, 0, "normal", 0.18});
    add({"M2_O2",    1.500,  0.365, false, 0, 0, "normal", 0.18});
    add({"M2_O3",    0.500,  0.365, false, 0, 0, "normal", 0.18});
    add({"M2_O4",   -0.500,  0.365, false, 0, 0, "normal", 0.18});
    add({"M2_O5",   -1.500,  0.365, false, 0, 0, "normal", 0.18});
    add({"M2_O6",   -2.500,  0.365, false, 0, 0, "normal", 0.18});
    add({"M2_O7",   -3.500,  0.365, false, 0, 0, "normal", 0.18});

    // ── Virage NORD au carrefour x=-4.562
    add({"M2_VN",   -4.562,  0.400, false, 0, 0, "normal", 0.12});
    add({"M2_P14",  -4.562,  0.435, false, 0, 0, "normal", 0.15});

    // ── Montée NORD x=-4.562 fixe
    add({"M2_NW1",  -4.562,  1.300, false, 0, 0, "normal", 0.18});
    add({"M2_NW2",  -4.562,  2.200, false, 0, 0, "normal", 0.18});
    add({"M2_NW3",  -4.562,  3.100, false, 0, 0, "normal", 0.18});
    add({"M2_NW4",  -4.562,  4.000, false, 0, 0, "normal", 0.18});
    add({"M2_P15",  -4.275,  4.700, false, 0, 0, "normal", 0.15});

    // ── Virage EST au carrefour (-4.175, 4.609)
    add({"M2_VE",   -4.175,  4.609, false, 0, 0, "normal", 0.12});

    // ── Ligne EST y=4.609 fixe (P15→P16)
    add({"M2_EH1",  -3.300,  4.609, false, 0, 0, "normal", 0.18});
    add({"M2_EH2",  -2.400,  4.609, false, 0, 0, "normal", 0.18});
    add({"M2_P16",  -1.583,  4.609, true, -1.583, 4.609, "normal", 0.15});

    // ── Virage gauche NORD au carrefour (-0.081, 4.609)
    add({"M2_VFN",  -0.081,  4.609, false, 0, 0, "normal", 0.12});
    add({"M2_P17",  -0.081,  5.369, false, 0, 0, "normal", 0.15});

    // ── Montée NORD vers fin x=0.300
    add({"M2_FN1",   0.100,  6.000, false, 0, 0, "normal", 0.18});
    add({"M2_FN2",   0.200,  7.000, false, 0, 0, "normal", 0.20});
    add({"M2_FIN",   0.300,  8.000, false, 0, 0, "normal", 0.20});

    // =======================================================
    // MISSIONS
    // =======================================================

    // ── Mission 1 : Ligne droite SUD→NORD
    missions_["test_ligne_droite"] = {
      "SUD_SPAWN",
      "SUD_S1", "SUD_S2", "SUD_S3",
      "SUD_S4", "SUD_S5", "SUD_S6",
      "SUD_RP_APPROCHE",
      "RP_P1", "RP_P2", "RP_P3", "RP_P4", "RP_P5",
      "RP_P6", "RP_P7", "RP_P8", "RP_P9",
      "NORD_RP_SORTIE",
      "NORD_S1", "NORD_S2", "NORD_S3",
      "NORD_S4", "NORD_S5", "NORD_FIN"
    };

    // ── Mission 2 : Tour partiel
    missions_["mission_2"] = {
      // SUD → virage EST
      "M2_P1", "M2_P1a","M2_P1b", "M2_P2", "M2_P3",
      // Ligne EST y=-5.397
      "M2_E1", "M2_E2", "M2_E3", "M2_E4", "M2_P4",
      // Virage NORD + montée x=5.330
      "M2_P5", "M2_N1", "M2_N2", "M2_P6",
      // Virage OUEST + ligne OUEST y=0.350
      "M2_W0", "M2_P7", "M2_W1", "M2_W2", "M2_P8", "M2_P9",
      // Rond-point — points mesurés
      "M2_RP_E", "M2_RP_A1", "M2_RP_A2",
      "M2_RP_A3", "M2_RP_A4", "M2_RP_A5",
      // Sortie RP → ligne OUEST y=0.365 (continue depuis M2_RP_A5)
      "M2_O6", "M2_O7",
      // Virage NORD x=-4.562
      "M2_VN", "M2_P14",
      // Montée NORD x=-4.562
      "M2_NW1", "M2_NW2", "M2_NW3", "M2_NW4", "M2_P15",
      // Virage EST + ligne EST y=4.609
      "M2_VE", "M2_EH1", "M2_EH2", "M2_P16",
      // Virage gauche NORD + fin
      "M2_VFN", "M2_P17",
      "M2_FN1", "M2_FN2", "M2_FIN"
    };
  }
};