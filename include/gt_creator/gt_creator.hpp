#ifndef GT_CREATOR_HPP_
#define GT_CREATOR_HPP_

#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"

#include "uwb_driver/msg/uwb_range.hpp"

typedef Eigen::Matrix<double, 3, 1> Vec3d;
typedef Eigen::VectorXd VecXd;
typedef Eigen::Matrix<double, 3, 3> Mat3d;
typedef Eigen::MatrixXd MatXd;

namespace navigation {
class gtCreator : public rclcpp::Node {
public:
  gtCreator();

  std::vector<Vec3d> anchor_positions_{std::vector<Vec3d>(20)};

  std::string uwb_topic_ = "/sensor/uwb/range";

  uint opt_mode = 1;
  // 0 : LS , 1 : GN
  //

private:
  std::vector<double> anchor_list_x_;
  std::vector<double> anchor_list_y_;
  std::vector<double> anchor_list_z_;

  std::vector<int64_t> anchor_id_lists_;

  uint num_anchors_;

  int id_matcher_[20] = {-1};

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
      gt_position_publisher_;

  rclcpp::Subscription<uwb_driver::msg::UwbRange>::SharedPtr uwb_subscriber_;

  void multilateration(const uwb_driver::msg::UwbRange::SharedPtr msg);
};

} // namespace navigation

#endif // !GT_CREATOR_HPP_
