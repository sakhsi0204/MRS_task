#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/transformer.h>

#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/UavState.h>
#include <mrs_msgs/PositionCommand.h>
#include <mrs_msgs/VelocityReferenceStamped.h>
#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/Reference.h>
#include <mrs_msgs/TrajectoryReference.h>

#include <uvdar_core/leader_tracker.h>

#include <leader_follower_task/FollowerConfig.h>

#include <random>

namespace leader_follower_task
{

  /* class Follower //{ */

  class Follower : public nodelet::Nodelet {

  public:
    virtual void onInit();

  private:
    ros::NodeHandle nh_;
    bool is_initialized_ = false;

    // A simple Kalman filter for tracking the leader's position and velocity
    std::unique_ptr<uvdar_core::LeaderTracker> leader_tracker_;
    std::mutex leader_tracker_mutex_;

    // UAV odometry
    mrs_lib::SubscribeHandler<mrs_msgs::UavState> sh_uav_state_;

    // UVDAR messages
    mrs_lib::SubscribeHandler<mrs_msgs::Vec4> sh_uvdar_;

    // Output command publishers
    ros::Publisher pub_position_cmd_;
    ros::Publisher pub_velocity_ref_;
    ros::Publisher pub_trajectory_ref_;

    // Timers
    ros::Timer timer_control_;

    // Parameters
    double _control_timer_rate_;
    Eigen::Vector3d _desired_offset_local_;
    double _prediction_horizon_;
    int _trajectory_points_;

    // Method to be called by the control timer
    void controlTimer(const ros::TimerEvent& event);
    
    // Generates the predictive trajectory
    mrs_msgs::TrajectoryReference generatePredictiveTrajectory(const Eigen::Vector3d& follower_pos, const Eigen::Vector3d& leader_pos, const Eigen::Vector3d& leader_vel);
  };

  //}

  /* onInit() //{ */

  void Follower::onInit() {

    nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

    ros::Time::waitForValid();

    mrs_lib::ParamLoader pl(nh_, "Follower");

    // Load general parameters
    pl.loadParam("control_timer_rate", _control_timer_rate_);
    pl.loadParam("desired_offset_local", _desired_offset_local_);

    // Load parameters for predictive control
    pl.loadParam("prediction_horizon", _prediction_horizon_);
    pl.loadParam("trajectory_points", _trajectory_points_);

    // Initialize the leader tracker (Kalman filter)
    leader_tracker_ = std::make_unique<uvdar_core::LeaderTracker>(nh_);

    // Subscribers
    mrs_lib::SubscribeHandlerOptions shopts;
    shopts.nh = nh_;
    shopts.node_name = "Follower";
    shopts.no_message_timeout = mrs_lib::no_timeout;
    shopts.threadsafe = true;
    shopts.autostart = true;
    shopts.queue_size = 10;

    sh_uav_state_ = mrs_lib::SubscribeHandler<mrs_msgs::UavState>(shopts, "uav_state_in");
    sh_uvdar_ = mrs_lib::SubscribeHandler<mrs_msgs::Vec4>(shopts, "uvdar_in", &Follower::controlTimer, this);

    // Publishers
    pub_position_cmd_ = nh_.advertise<mrs_msgs::PositionCommand>("position_cmd_out", 1);
    pub_velocity_ref_ = nh_.advertise<mrs_msgs::VelocityReferenceStamped>("velocity_ref_out", 1);
    pub_trajectory_ref_ = nh_.advertise<mrs_msgs::TrajectoryReference>("trajectory_ref_out", 1);
    
    // Timers
    timer_control_ = nh_.createTimer(ros::Rate(_control_timer_rate_), &Follower::controlTimer, this, false, false);
    
    is_initialized_ = true;

    ROS_INFO("[Follower]: initialized");
  }

  //}

  /* controlTimer() //{ */

  void Follower::controlTimer([[maybe_unused]] const ros::TimerEvent& event) {

    if (!is_initialized_) {
      return;
    }

    if (!sh_uvdar_.hasMsg()) {
      ROS_WARN_THROTTLE(1.0, "[Follower]: Waiting for first UVDAR message.");
      return;
    }

    if (!sh_uav_state_.hasMsg()){
      ROS_WARN_THROTTLE(1.0, "[Follower]: Waiting for first odometry message.");
      return;
    }

    // Update the leader tracker with the latest UVDAR measurement
    auto uvdar_msg = sh_uvdar_.getMsg();
    {
      std::scoped_lock lock(leader_tracker_mutex_);
      leader_tracker_->processUvDar(uvdar_msg);
    }
    
    // Get the smoothed state estimate of the leader
    auto leader_estimate = leader_tracker_->getCurrentEstimate();
    if (!leader_estimate.has_value()) {
      ROS_WARN_THROTTLE(1.0, "[Follower]: Kalman filter does not have a valid estimate yet.");
      return;
    }
    
    Eigen::Vector3d leader_position(leader_estimate.value().position.x, leader_estimate.value().position.y, leader_estimate.value().position.z);
    Eigen::Vector3d leader_velocity(leader_estimate.value().velocity.x, leader_estimate.value().velocity.y, leader_estimate.value().velocity.z);

    // Get follower's current state
    mrs_msgs::UavState uav_state = *sh_uav_state_.getMsg();
    Eigen::Vector3d follower_position(uav_state.pose.position.x, uav_state.pose.position.y, uav_state.pose.position.z);
    
    // Generate and publish the predictive trajectory
    mrs_msgs::TrajectoryReference trajectory_cmd = generatePredictiveTrajectory(follower_position, leader_position, leader_velocity);
    pub_trajectory_ref_.publish(trajectory_cmd);
  }
  
  //}

  /* generatePredictiveTrajectory() //{ */

  mrs_msgs::TrajectoryReference Follower::generatePredictiveTrajectory(const Eigen::Vector3d& follower_pos, const Eigen::Vector3d& leader_pos, const Eigen::Vector3d& leader_vel) {
      
      mrs_msgs::TrajectoryReference trajectory_cmd;
      trajectory_cmd.header.stamp = ros::Time::now();
      trajectory_cmd.header.frame_id = sh_uav_state_.getMsg()->header.frame_id;
      trajectory_cmd.use_for_control = true;
      trajectory_cmd.fly_now = true;

      double dt = _prediction_horizon_ / _trajectory_points_;

      for (int i = 0; i < _trajectory_points_; ++i) {
          double t = (i + 1) * dt;
          
          // Predict leader's future position
          Eigen::Vector3d predicted_leader_pos = leader_pos + leader_vel * t;
          
          // Calculate desired follower position with offset
          Eigen::Vector3d desired_follower_pos = predicted_leader_pos + _desired_offset_local_;

          mrs_msgs::Reference point;
          point.position.x = desired_follower_pos.x();
          point.position.y = desired_follower_pos.y();
          point.position.z = desired_follower_pos.z();
          point.heading = atan2(leader_vel.y(), leader_vel.x()); // Point towards leader's direction of travel

          trajectory_cmd.points.push_back(point);
      }
      return trajectory_cmd;
  }
  
  //}

}  // namespace leader_follower_task

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(leader_follower_task::Follower, nodelet::Nodelet)
