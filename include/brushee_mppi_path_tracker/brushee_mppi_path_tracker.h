//  Copyright 2025 amsl

#ifndef BRUSHEE_MPPI_PATH_TRACKER_BRUSHEE_MPPI_PATH_TRACKER_BRUSHEE_MPPI_PATH_TRACKER_H
#define BRUSHEE_MPPI_PATH_TRACKER_BRUSHEE_MPPI_PATH_TRACKER_BRUSHEE_MPPI_PATH_TRACKER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <random>
#include <vector>


class RobotStates
{
  public:
    RobotStates();
    ~RobotStates();
    void init(int horizon);
    std::vector<double> x_, y_, yaw_;
    std::vector<double> vx_, vy_, w_;
};

RobotStates::RobotStates()
{
}

RobotStates::~RobotStates()
{
}

void RobotStates::init(int horizon)
{
  x_.resize(horizon);
  y_.resize(horizon);
  yaw_.resize(horizon);
  vx_.resize(horizon);
  vy_.resize(horizon);
  w_.resize(horizon);
  for (int i=0; i<horizon; i++)
  {
    x_[i] = 0.0;
    y_[i] = 0.0;
    yaw_[i] = 0.0;
    vx_[i] = 0.0;
    vy_[i] = 0.0;
    w_[i] = 0.0;
  }
}

class BrusheeMppiPathTracker
{
  public:
    BrusheeMppiPathTracker();
    ~BrusheeMppiPathTracker();
    void process();
  private:
    void path_callback(const geometry_msgs::PoseArray::ConstPtr& msg);
    void robot_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void sampling();
    void clamp(double& value, double min, double max);
    void predict_states();
    void predict_next_states(RobotStates &state, int t);
    void calc_weights();
    void calc_ref_path();
    void determine_optimal_solution();
    void publish();
    double calc_cost(RobotStates &state);
    double calc_min_dist(RobotStates &state, std::vector<double>& x_ref, std::vector<double>& y_ref);
    double calc_min_angle_dist(RobotStates &state, std::vector<double>& yaw_ref);
    double calc_omega(double x, double y, double yaw, double x_goal, double y_goal, double yaw_goal);
    int get_current_index();

    int horizon_;
    int current_index_;
    int hz_;
    int num_samples_;
    double control_noise_;
    double vx_max_, vy_max_, w_max_;
    double vx_min_, vy_min_, w_min_;
    double vx_ref_, vy_ref_, w_ref_;
    double lambda_;
    double path_weight_;
    double vel_weight_;
    double angle_weight_;
    double current_time_;
    double dt_;
    double resolution_;

    bool is_path_;
    bool is_robot_pose_;

    std::vector<double> x_ref_, y_ref_, yaw_ref_;
    std::vector<RobotStates> samples_;
    std::vector<double> weights_;

    RobotStates optimal_solution_;
    geometry_msgs::PoseArray path_;
    geometry_msgs::PoseWithCovarianceStamped robot_pose_;
    geometry_msgs::Twist cmd_vel_;
    // tf2_ros::Buffer tf_Buffer_;
    // tf2_ros::TransformListener tflistener_{tf_Buffer_};

    //  for debug
    void publish_path();
    void publish_ref_path();
    void publish_optimal_path();
    void publish_candidate_path();
    void save_data();
    nav_msgs::Path path_msg_;
    nav_msgs::Path ref_path_msg_;
    nav_msgs::Path optimal_path_msg_;
    visualization_msgs::MarkerArray candidate_path_msg_;
    ros::Publisher path_pub_;
    ros::Publisher ref_path_pub_;
    ros::Publisher optimal_path_pub_;
    ros::Publisher candidate_path_pub_;

    ros::Time last_time_;
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber path_sub_;
    ros::Subscriber robot_pose_sub_;
    ros::Publisher cmd_vel_pub_;

};

#endif  // BRUSHEE_MPPI_PATH_TRACKER_BRUSHEE_MPPI_PATH_TRACKER_BRUSHEE_MPPI_PATH_TRACKER_H
