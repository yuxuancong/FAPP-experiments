#include <ros/ros.h>
#include <obj_state_msgs/ObjectsStates.h>
#include <obj_state_msgs/State.h>
#include <unordered_map>
#include <string>
#include <iostream>
#include <fstream>
#include "kf_compare.hpp"

struct TrackerPair {
  StandardKF::Ptr skf;
  AdaptiveKF::Ptr akf;
};

class KFCompareNode {
public:
  KFCompareNode(ros::NodeHandle& nh, ros::NodeHandle& pnh): nh_(nh), pnh_(pnh) {
    // 从私有参数读取配置
    pnh_.param<std::string>("gt_topic", gt_topic_, std::string("/map_generator/obj_gt"));
    pnh_.param<std::string>("est_topic", est_topic_, std::string("/states"));
    pnh_.param("dt", dt_, 0.02);
    pnh_.param<std::string>("output_csv", output_csv_, std::string("/tmp/kf_compare_log.csv"));
    
    // 使用全局 NodeHandle 订阅绝对话题
    gt_sub_ = nh_.subscribe(gt_topic_, 10, &KFCompareNode::gtCallback, this);
    est_sub_ = nh_.subscribe(est_topic_, 10, &KFCompareNode::estCallback, this);
    report_timer_ = nh_.createTimer(ros::Duration(1.0), &KFCompareNode::reportCallback, this);
    
    openCsv();
    ROS_INFO_STREAM("[KFCompare] node started, dt=" << dt_ << ", csv=" << output_csv_
                    << ", gt_topic=" << gt_topic_ << ", est_topic=" << est_topic_);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber gt_sub_, est_sub_;
  ros::Timer report_timer_;
  double dt_;

  ros::Time last_gt_, last_est_;

  std::string output_csv_;
  std::ofstream csv_;
  bool csv_ok_ = false;

  std::string gt_topic_, est_topic_;

  // id -> GT
  std::unordered_map<int, obj_state_msgs::State> gt_map_;
  // id -> trackers
  std::unordered_map<int, TrackerPair> trackers_;

  double rmse_skf_pos_sum_ = 0, rmse_akf_pos_sum_ = 0;
  int rmse_count_ = 0;

  void gtCallback(const obj_state_msgs::ObjectsStates::ConstPtr& msg) {
    last_gt_ = ros::Time::now();
    gt_map_.clear();
    for (size_t i = 0; i < msg->states.size(); ++i) {
      // 使用索引作为id，避免header.seq为0导致匹配不到
      gt_map_[static_cast<int>(i)] = msg->states[i];
    }
  }

  void estCallback(const obj_state_msgs::ObjectsStates::ConstPtr& msg) {
    last_est_ = ros::Time::now();
    for (size_t i = 0; i < msg->states.size(); ++i) {
      const auto& est = msg->states[i];
      int id = static_cast<int>(i); // 与gt使用相同的索引做匹配
      ensureTracker(id, est);

      Eigen::Vector3d z_pos(est.position.x, est.position.y, est.position.z);
      Eigen::Vector3d z_vel(est.velocity.x, est.velocity.y, est.velocity.z);

      trackers_[id].skf->predict();
      trackers_[id].akf->predict();
      trackers_[id].skf->update(z_pos, z_vel);
      trackers_[id].akf->update(z_pos, z_vel);

      auto it_gt = gt_map_.find(id);
      if (it_gt != gt_map_.end()) {
        Eigen::Vector3d gt_pos(it_gt->second.position.x, it_gt->second.position.y, it_gt->second.position.z);
        double err_skf = (trackers_[id].skf->pos() - gt_pos).norm();
        double err_akf = (trackers_[id].akf->pos() - gt_pos).norm();
        rmse_skf_pos_sum_ += err_skf * err_skf;
        rmse_akf_pos_sum_ += err_akf * err_akf;
        rmse_count_++;
      }
    }
  }

  void ensureTracker(int id, const obj_state_msgs::State& est) {
    if (trackers_.find(id) != trackers_.end()) return;
    Eigen::Vector3d z_pos(est.position.x, est.position.y, est.position.z);
    TrackerPair tp;
    tp.skf = std::make_shared<StandardKF>(dt_);
    tp.akf = std::make_shared<AdaptiveKF>(dt_, 10);
    tp.skf->reset(z_pos, id);
    tp.akf->reset(z_pos, id);
    trackers_[id] = tp;
  }

  void reportCallback(const ros::TimerEvent&) {
    const double timeout = 1.5;
    ros::Time now = ros::Time::now();
    if (!last_gt_.isZero() && (now - last_gt_).toSec() > timeout) {
      ROS_WARN_THROTTLE(1.0, "[KFCompare] no ground-truth messages recently (%s)", gt_topic_.c_str());
    }
    if (!last_est_.isZero() && (now - last_est_).toSec() > timeout) {
      ROS_WARN_THROTTLE(1.0, "[KFCompare] no estimate messages recently (%s)", est_topic_.c_str());
    }
    if (rmse_count_ == 0) return;
    double rmse_skf = std::sqrt(rmse_skf_pos_sum_ / rmse_count_);
    double rmse_akf = std::sqrt(rmse_akf_pos_sum_ / rmse_count_);
    ROS_INFO_STREAM("[KFCompare] RMSE_pos standard=" << rmse_skf << ", adaptive=" << rmse_akf
                    << " diff=" << (rmse_akf - rmse_skf));

    // 追加写入CSV：time,rmse_skf,rmse_akf,count
    if (csv_ok_) {
      csv_ << now.toSec() << "," << rmse_skf << "," << rmse_akf << "," << rmse_count_ << "\n";
      csv_.flush();
    }
  }

  void openCsv() {
    csv_.open(output_csv_, std::ios::out | std::ios::app);
    if (!csv_.is_open()) {
      ROS_WARN_STREAM("[KFCompare] cannot open csv file: " << output_csv_);
      csv_ok_ = false;
      return;
    }
    // 若文件为空则写表头
    csv_.seekp(0, std::ios::end);
    if (csv_.tellp() == 0) {
      csv_ << "time,rmse_standard,rmse_adaptive,count\n";
      csv_.flush();
    }
    csv_ok_ = true;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "kf_compare_node");
  ros::NodeHandle nh;       // global handle for topics
  ros::NodeHandle pnh("~"); // private handle for parameters
  KFCompareNode node(nh, pnh);
  ros::spin();
  return 0;
}
