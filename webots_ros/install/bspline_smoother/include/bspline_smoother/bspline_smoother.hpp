// bspline_smoother.hpp
// Deklariert das BSplineSmoother-Plugin, das das nav2_core::Smoother-
// Interface implementiert und via pluginlib registriert wird.
//
// Quellen:
//   [NAV2_SMOOTHER]     Open Navigation LLC (2024). Smoother Server – Nav2.
//                       https://docs.nav2.org/configuration/packages/
//                       configuring-smoother-server.html
//   [NAV2_CORE_SRC]     ros-navigation (2024). nav2_core/smoother.hpp.
//                       https://github.com/ros-navigation/navigation2/blob/
//                       main/nav2_core/include/nav2_core/smoother.hpp

#ifndef BSPLINE_SMOOTHER__BSPLINE_SMOOTHER_HPP_
#define BSPLINE_SMOOTHER__BSPLINE_SMOOTHER_HPP_

#include <memory>
#include <string>

#include "nav2_core/smoother.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

#include "bspline_smoother/bspline.hpp"

namespace bspline_smoother {

// ---------------------------------------------------------------------------
// BSplineSmoother
// Nav2-Plugin (nav2_core::Smoother). Glättet einen Nav2-Pfad via
// interpolierendem kubischem B-Spline und prüft den Ausgabepfad
// gegen die Costmap auf Kollisionsfreiheit.
//
// Parameter (nav2_params.yaml):
//   output_samples             (int,    default: 100)  – Ausgabepunkte.
//   collision_check_resolution (double, default: 0.1)  – Prüfabstand (m).
//   max_control_points         (int,    default: 60)   – Max. Stuetzpunkte.
// ---------------------------------------------------------------------------
class BSplineSmoother : public nav2_core::Smoother {
 public:
  BSplineSmoother() = default;
  ~BSplineSmoother() override = default;

  // nav2_core::Smoother-Interface – Lifecycle [NAV2_SMOOTHER].
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub)
  override;

  void cleanup()    override;
  void activate()   override;
  void deactivate() override;

  // Hauptmethode: glättet path in-place.
  // Gibt false zurück, wenn der geglättete Pfad kollidiert.
  bool smooth(
    nav_msgs::msg::Path& path,
    const rclcpp::Duration& max_time)
  override;

 private:
  // Prüft einen Weltpunkt gegen die aktuelle Costmap.
  bool IsCollisionFree(const Point2D& point);

  // Lifecycle-State.
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("BSplineSmoother")};
  std::string name_;

  // Nav2-Costmap-Interfaces.
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>  costmap_sub_;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;

  // Plugin-Parameter.
  int    output_samples_{100};
  double collision_check_resolution_{0.1};
  int    max_control_points_{60};
};

}  // namespace bspline_smoother

#endif  // BSPLINE_SMOOTHER__BSPLINE_SMOOTHER_HPP_