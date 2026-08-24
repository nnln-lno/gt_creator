#include "gt_creator/gt_creator.hpp"
#include <functional>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace navigation {
gtCreator::gtCreator() : Node("gt_creator_node") {
  this->declare_parameter("gt_anchor_id_lists", std::vector<int>{});
  this->declare_parameter("anchor_pos_x", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("anchor_pos_y", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("anchor_pos_z", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("num_anchors", 0);
  this->declare_parameter("uwb_topic", "/sensor/uwb/range");

  // ----------------------------------------------------------------------------
  this->get_parameter("anchor_pos_x", anchor_list_x_);
  this->get_parameter("anchor_pos_y", anchor_list_y_);
  this->get_parameter("anchor_pos_z", anchor_list_z_);
  this->get_parameter("uwb_topic", uwb_topic_);

  try {
    anchor_id_lists_ =
        this->get_parameter("anchor_id_lists").as_integer_array();
  } catch (const rclcpp::exceptions::ParameterNotDeclaredException &e) {
    RCLCPP_ERROR(this->get_logger(),
                 "Anchor ID list parameter not declared. Check Config Again");
    throw e;
  }

  num_anchors_ = anchor_id_lists_.size();

  for (uint i = 0; i < num_anchors_; i++) {
    uint uidx = anchor_id_lists_[i] - 1;

    anchor_positions_[uidx] =
        Vec3d(anchor_list_x_[uidx], anchor_list_y_[uidx], anchor_list_z_[uidx]);
  }

  gt_position_publisher_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>("/gt/position",
                                                               100);

  uwb_subscriber_ = this->create_subscription<uwb_driver::msg::UwbRange>(
      uwb_topic_, 10, std::bind(&gtCreator::multilateration, this, _1));

  if (!opt_mode) {
    RCLCPP_INFO(this->get_logger(),
                "Ready to create pseudo ground truth position data using uwb "
                "sensor. (Least Square Method)");
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Ready to create pseudo ground truth position data using uwb "
                "sensor. (Gauss Newton Method)");
  }
}

void gtCreator::multilateration(
    const uwb_driver::msg::UwbRange::SharedPtr msg) {
  Vec3d opt_pos = Vec3d(0.0, 0.0, -1.0);

  std::vector<float> dist = msg->dist;
  std::vector<int> anc_id = msg->anchor_ids;
  std::vector<float> temp_res;

  int m_size = (int)msg->dist.size();

  for (int i = 0; i < m_size; i++) {
    dist[i] = msg->dist[i] * 1e-2;
    anc_id[i] = msg->anchor_ids[i] - 1;
  }

  std::vector<int> idx(anc_id.size());
  std::iota(idx.begin(), idx.end(), 0);

  std::sort(idx.begin(), idx.end(),
            [&dist](int i1, int i2) { return dist[i1] < dist[i2]; });

  Vec3d base_p = anchor_positions_[anc_id[0]];

  geometry_msgs::msg::PointStamped gt_estimated_position_;

  if (!opt_mode) {
    if (m_size >= 4) {
      MatXd A(m_size, 3);
      VecXd B(m_size, 1);

      for (int i = 0; i < m_size - 1; i++) {
        // std::cout << anchor_positions_[anc_id[i + 1]].transpose() <<
        // std::endl;
        A.row(i) = 2 * (anchor_positions_[anc_id[i + 1]] - base_p).transpose();
        B(i) = (anchor_positions_[anc_id[i + 1]].squaredNorm() -
                base_p.squaredNorm()) -
               (dist[i + 1] * dist[i + 1] - dist[0] * dist[0]);
      }

      std::cout << A << std::endl;
      std::cout << B.transpose() << std::endl;

      opt_pos = (A.transpose() * A).inverse() * A.transpose() * B;

      std::cout << "Pseudo Position : " << opt_pos.transpose() << std::endl;

      gt_estimated_position_.header.frame_id = "map";
      gt_estimated_position_.header.stamp = this->get_clock()->now();

      gt_estimated_position_.point.x = opt_pos.x();
      gt_estimated_position_.point.y = opt_pos.y();
      gt_estimated_position_.point.z = opt_pos.z();

      gt_position_publisher_->publish(gt_estimated_position_);
    }
  } else {
    if (m_size >= 4) {
      double term_cond = 1e-5;
      double min_cond = 1e-1;
      // double min_cond = 0.05;

      double lamb = 0.5;
      int iter = 200;

      VecXd err_arr(iter);
      MatXd pos_arr(3, iter);

      for (int opt = 0; opt < iter; opt++) {
        MatXd J(m_size, 3);
        VecXd res(m_size);

        // Use most 4 closest anchors for multilateration
        for (int i = 0; i < m_size; i++) {
          // std::cout << "We got : " << anc_id[idx[i]] << " with position :" <<
          // anchor_positions_[anc_id[idx[i]]].transpose() << std::endl;
          J.row(i) = (anchor_positions_[anc_id[idx[i]]] - opt_pos).transpose() /
                     (anchor_positions_[anc_id[idx[i]]] - opt_pos).norm();
          res(i) = dist[idx[i]] -
                   (anchor_positions_[anc_id[idx[i]]] - opt_pos).norm();
        }

        Vec3d d_pos = (J.transpose() * J + lamb * Mat3d::Identity()).inverse() *
                      (J.transpose() * res);

        opt_pos = opt_pos - d_pos;

        err_arr(opt) = res.norm();
        pos_arr.col(opt) = opt_pos;

        if (opt > 0) {
          if ((abs(err_arr(opt) - err_arr(opt - 1)) < term_cond) ||
              (err_arr(opt) < min_cond)) {
            gt_estimated_position_.header.frame_id = "map";
            gt_estimated_position_.header.stamp = this->get_clock()->now();

            gt_estimated_position_.point.x = opt_pos.x();
            gt_estimated_position_.point.y = opt_pos.y();
            gt_estimated_position_.point.z = opt_pos.z();

            gt_position_publisher_->publish(gt_estimated_position_);

            std::cout << "Pseudo Position : " << opt_pos.transpose() << std::endl;
            
            return;
          }

          if (err_arr(opt) < err_arr(opt - 1)) {
            lamb = std::max(lamb / 9,
                            1e-7); // Decrease lambda if error is decreasing
          } else {
            lamb = std::min(lamb * 9,
                            1e+7); // Increase lambda if error is increasing
          }
        }
      }

      int min_idx;
      double min_err = err_arr.minCoeff(&min_idx);

      if (min_err < 1.0) {
        gt_estimated_position_.header.frame_id = "map";
        gt_estimated_position_.header.stamp = this->get_clock()->now();

        gt_estimated_position_.point.x = pos_arr(0, min_idx);
        gt_estimated_position_.point.y = pos_arr(1, min_idx);
        gt_estimated_position_.point.z = pos_arr(2, min_idx);

        gt_position_publisher_->publish(gt_estimated_position_);
      }
    }
  }
}
} // namespace navigation
