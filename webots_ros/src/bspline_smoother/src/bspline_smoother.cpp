// bspline_smoother.cpp
// Implementiert das nav2_core::Smoother-Interface als pluginlib-Plugin.
// Dient als Wrapper zwischen dem Nav2 Smoother Server und der
// mathematischen B-Spline-Implementierung in bspline.cpp/.hpp.
//
// Quellen (Plugin-Architektur):
//   [NAV2_SMOOTHER]     Open Navigation LLC (2024). Smoother Server – Nav2.
//                       https://docs.nav2.org/configuration/packages/
//                       configuring-smoother-server.html
//   [NAV2_ADD_SMOOTHER] Open Navigation LLC (2024). Adding a Smoother to a BT.
//                       https://docs.nav2.org/tutorials/docs/
//                       adding_smoother.html
//   [NAV2_CORE_SRC]     ros-navigation (2024). nav2_smoother.cpp.
//                       https://github.com/ros-navigation/navigation2/blob/
//                       main/nav2_smoother/src/nav2_smoother.cpp

#include "bspline_smoother/bspline_smoother.hpp"

#include <pluginlib/class_list_macros.hpp>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"

PLUGINLIB_EXPORT_CLASS(bspline_smoother::BSplineSmoother, nav2_core::Smoother)

namespace bspline_smoother {

// ---------------------------------------------------------------------------
// configure – Lifecycle-Konfigurationsphase [NAV2_SMOOTHER].
// Deklariert und liest Plugin-Parameter aus der nav2_params.yaml.
//   output_samples:             Anzahl Wegpunkte im geglätteten Ausgabepfad.
//   collision_check_resolution: Mindestabstand zwischen Kollisionsprüfpunkten (m).
// ---------------------------------------------------------------------------
void BSplineSmoother::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> /*tf*/,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub) {
  auto node = parent.lock();
  node_          = parent;
  name_          = name;
  costmap_sub_   = costmap_sub;
  footprint_sub_ = footprint_sub;
  logger_        = node->get_logger();

  RCLCPP_INFO(logger_, "Configuring B-Spline Smoother: %s", name_.c_str());

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".output_samples", rclcpp::ParameterValue(100));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".collision_check_resolution", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_control_points", rclcpp::ParameterValue(60));

  node->get_parameter(name_ + ".output_samples",             output_samples_);
  node->get_parameter(name_ + ".collision_check_resolution", collision_check_resolution_);
  node->get_parameter(name_ + ".max_control_points",         max_control_points_);

  RCLCPP_INFO(logger_,
    "B-Spline Smoother ready: output_samples=%d, "
    "collision_check_resolution=%.3f, max_control_points=%d",
    output_samples_, collision_check_resolution_, max_control_points_);
}

// Lifecycle-Callbacks – keine Ressourcen zu verwalten.
void BSplineSmoother::cleanup()    { RCLCPP_INFO(logger_, "cleanup()");    }
void BSplineSmoother::activate()   { RCLCPP_INFO(logger_, "activate()");   }
void BSplineSmoother::deactivate() { RCLCPP_INFO(logger_, "deactivate()"); }

// ---------------------------------------------------------------------------
// IsCollisionFree – Prüft einen 2D-Weltpunkt gegen die Costmap.
// Gibt false zurück, wenn der Punkt außerhalb der Karte liegt oder
// der Kostenwert ≥ INSCRIBED_INFLATED_OBSTACLE ist.
// ---------------------------------------------------------------------------
bool BSplineSmoother::IsCollisionFree(const Point2D& point) {
  auto costmap = costmap_sub_->getCostmap();
  unsigned int mx, my;
  if (!costmap->worldToMap(point.x, point.y, mx, my)) {
    return false;
  }
  return costmap->getCost(mx, my) <
         nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

// ---------------------------------------------------------------------------
// smooth – Hauptmethode des nav2_core::Smoother-Interfaces [NAV2_SMOOTHER].
// Pipeline:
//   1. Deduplizierung zu naher Wegpunkte (Threshold: 1e-6 m).
//   2. Kubischer B-Spline über alle einzigartigen Datenpunkte.
//   3. Kollisionsprüfung des geglätteten Pfads gegen die Costmap.
//   4. Orientierungen via Forward-Differenz (Tangente); letzter Punkt
//      übernimmt die Zielorientierung des Originalpfads.
// ---------------------------------------------------------------------------
bool BSplineSmoother::smooth(
    nav_msgs::msg::Path& path,
    const rclcpp::Duration& /*max_time*/) {
  const size_t raw_size = path.poses.size();

  if (raw_size < 4) {
    RCLCPP_WARN(logger_,
      "Path too short (%zu poses) – pass-through.", raw_size);
    return true;
  }

  // Schritt 1: Deduplizierung.
  std::vector<Point2D> data_pts;
  data_pts.reserve(raw_size);
  for (const auto& ps : path.poses) {
    const Point2D p{ps.pose.position.x, ps.pose.position.y};
    if (data_pts.empty()) {
      data_pts.push_back(p);
    } else {
      const double dx = p.x - data_pts.back().x;
      const double dy = p.y - data_pts.back().y;
      if (std::hypot(dx, dy) > 1e-6) {
        data_pts.push_back(p);
      }
    }
  }

  if (data_pts.size() < 4) {
    RCLCPP_WARN(logger_,
      "Too few unique points after deduplication – pass-through.");
    return true;
  }

  // Schritt 1b: Downsampling bei langen Pfaden.
  // Reduziert die Stuetzpunktmenge auf max_control_points_, um die
  // O(n^3)-QR-Zerlegung handhabbar zu halten. Start- und Endpunkt
  // bleiben erhalten; dazwischen wird gleichmaessig abgetastet.
  if (static_cast<int>(data_pts.size()) > max_control_points_) {
    const size_t orig_size = data_pts.size();
    std::vector<Point2D> downsampled;
    downsampled.reserve(max_control_points_);

    const double step =
        static_cast<double>(orig_size - 1) / (max_control_points_ - 1);

    for (int i = 0; i < max_control_points_ - 1; ++i) {
      downsampled.push_back(data_pts[static_cast<size_t>(i * step)]);
    }
    downsampled.push_back(data_pts.back());

    data_pts = downsampled;
    RCLCPP_INFO(logger_,
      "Downsampled: %zu -> %zu control points.", orig_size, data_pts.size());
  }

  try {
    // Schritt 2: B-Spline interpolieren und abtasten.
    BSpline spline(data_pts, /*degree=*/3);
    const auto smoothed = spline.Sample(output_samples_);

    // Schritt 3: Kollisionsprüfung.
    for (const auto& p : smoothed) {
      if (!IsCollisionFree(p)) {
        RCLCPP_WARN(logger_,
          "Smoothed path collides at (%.2f, %.2f) – keeping original.",
          p.x, p.y);
        return false;
      }
    }

    // Schritt 4: Ausgabepfad mit Tangenten-Orientierungen aufbauen.
    nav_msgs::msg::Path out;
    out.header = path.header;
    out.poses.reserve(smoothed.size());

    for (size_t i = 0; i < smoothed.size(); ++i) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header          = path.header;
      ps.pose.position.x = smoothed[i].x;
      ps.pose.position.y = smoothed[i].y;
      ps.pose.position.z = 0.0;

      if (i < smoothed.size() - 1) {
        const double yaw = std::atan2(
          smoothed[i + 1].y - smoothed[i].y,
          smoothed[i + 1].x - smoothed[i].x);
        ps.pose.orientation.z = std::sin(yaw * 0.5);
        ps.pose.orientation.w = std::cos(yaw * 0.5);
      } else {
        // Letzter Punkt: Zielorientierung aus Originalpfad übernehmen.
        ps.pose.orientation = path.poses.back().pose.orientation;
      }
      out.poses.push_back(ps);
    }

    path = out;
    RCLCPP_INFO(logger_,
      "Smoothing OK: %zu data points → %zu output points.",
      data_pts.size(), out.poses.size());
    return true;

  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "B-Spline failed: %s", e.what());
    return false;
  }
}

}  // namespace bspline_smoother