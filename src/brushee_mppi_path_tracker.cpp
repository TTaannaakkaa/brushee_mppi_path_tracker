//  Copyright 2025 amsl

#include "brushee_mppi_path_tracker/brushee_mppi_path_tracker.h"

BrusheeMppiPathTracker::BrusheeMppiPathTracker() : private_nh_("~")
{
  private_nh_.param("hz", hz_, 10);
  private_nh_.param("horizon", horizon_, 15);
  private_nh_.param("num_samples", num_samples_, 1000);
  private_nh_.param("control_noise", control_noise_, 0.4);
  private_nh_.param("lambda", lambda_, 1.0);
  private_nh_.param("vx_max", vx_max_, 0.3);
  private_nh_.param("vy_max", vy_max_, 0.3);
  private_nh_.param("w_max", w_max_, 1.0);
  private_nh_.param("vx_min", vx_min_, 0.0);
  private_nh_.param("vy_min", vy_min_, -0.3);
  private_nh_.param("w_min", w_min_, -1.0);
  private_nh_.param("vx_ref", vx_ref_, 0.1);
  private_nh_.param("vy_ref", vy_ref_, 0.1);
  private_nh_.param("w_ref", w_ref_, 0.0);
  private_nh_.param("resolution", resolution_, 0.05);
  private_nh_.param("path_weight", path_weight_, 1.0);
  private_nh_.param("vel_weight", vel_weight_, 1.0);
  private_nh_.param("angle_weight", angle_weight_, 1.0);
  private_nh_.param("is_angler", is_angler_, false);

  ROS_INFO_STREAM("hz: " << hz_);
  ROS_INFO_STREAM("horizon: " << horizon_);
  ROS_INFO_STREAM("num_samples: " << num_samples_);
  ROS_INFO_STREAM("control_noise: " << control_noise_);
  ROS_INFO_STREAM("lambda: " << lambda_);
  ROS_INFO_STREAM("vx_max: " << vx_max_);
  ROS_INFO_STREAM("vy_max: " << vy_max_);
  ROS_INFO_STREAM("w_max: " << w_max_);
  ROS_INFO_STREAM("vx_min: " << vx_min_);
  ROS_INFO_STREAM("vy_min: " << vy_min_);
  ROS_INFO_STREAM("w_min: " << w_min_);
  ROS_INFO_STREAM("vx_ref: " << vx_ref_);
  ROS_INFO_STREAM("vy_ref: " << vy_ref_);
  ROS_INFO_STREAM("w_ref: " << w_ref_);
  ROS_INFO_STREAM("resolution: " << resolution_);
  ROS_INFO_STREAM("path_weight: " << path_weight_);
  ROS_INFO_STREAM("vel_weight: " << vel_weight_);
  ROS_INFO_STREAM("angle_weight: " << angle_weight_);


  path_sub_ = nh_.subscribe("/correct_footprints_yaw", 1, &BrusheeMppiPathTracker::path_callback, this);
  robot_pose_sub_ = nh_.subscribe("/amcl_pose", 1, &BrusheeMppiPathTracker::robot_pose_callback, this);
  integrated_pose_sub_ = nh_.subscribe("/integrated_pose", 1, &BrusheeMppiPathTracker::integrated_pose_callback, this);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/brushee/cmd_vel", 1);
  //  for debug
  path_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/brushee/mppi/path", 1);
  ref_path_pub_ = nh_.advertise<nav_msgs::Path>("/brushee/mppi/ref_path", 1);
  optimal_path_pub_ = nh_.advertise<nav_msgs::Path>("/brushee/mppi/optimal_path", 1);
  candidate_path_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/brushee/mppi/candidate_path", 1);

  samples_.resize(num_samples_);
  for (int i=0; i<num_samples_; i++)
    samples_[i].init(horizon_);
  optimal_solution_.init(horizon_);
  x_ref_.resize(horizon_);
  y_ref_.resize(horizon_);
  yaw_ref_.resize(horizon_);
  weights_.resize(num_samples_);
  pre_vx_.resize(num_samples_);

  last_time_ = ros::Time(0);
  pub_idx_ = 0;
  is_path_ = false;
  is_robot_pose_ = false;
  is_first_ = false;
}

BrusheeMppiPathTracker::~BrusheeMppiPathTracker()
{
}

void BrusheeMppiPathTracker::path_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  path_ = *msg;
  is_path_ = true;
}

void BrusheeMppiPathTracker::robot_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  robot_pose_ = *msg;
  is_first_ = true;
  is_robot_pose_ = true;
}

void BrusheeMppiPathTracker::integrated_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  integrated_pose_ = *msg;
  is_first_ = true;
  is_robot_pose_ = true;
}

void BrusheeMppiPathTracker::process()
{
  ros::Rate loop_rate(hz_);
  while (ros::ok())
  {
    if (is_path_ && is_first_)
    {
      if (last_time_ == ros::Time(0))
      {
        last_time_ = ros::Time::now();
        continue;
      }
      else
      {
        dt_ = (ros::Time::now() - last_time_).toSec();
        last_time_ = ros::Time::now();
        ROS_INFO_STREAM_THROTTLE(1.0, "dt: " << dt_);
        if (is_robot_pose_)
        {
          calc_ref_path();
          sampling();  //  TODO 自己位置推定の遅れを考慮する（optimal_path の次を用いるとか？）
          predict_states();
          calc_weights();
          determine_optimal_solution();
          pub_idx_ = 0;
          is_robot_pose_ = false;
        }
        else
          pub_idx_++;
        publish();
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void BrusheeMppiPathTracker::sampling() // ランダムに制御指令をサンプリング 
{
  std::random_device rnd;
  std::mt19937 mt(rnd());

  for (int t=0; t<horizon_-1; t++)
  {
    std::normal_distribution<> norm_vx(optimal_solution_.vx_[t], control_noise_);
    std::normal_distribution<> norm_vy(optimal_solution_.vy_[t], control_noise_);
    std::normal_distribution<> norm_w(optimal_solution_.w_[t], control_noise_);

    for (int i=0; i<num_samples_; i++)
    {
      samples_[i].vx_[t] = norm_vx(mt);
      samples_[i].vy_[t] = norm_vy(mt);
      samples_[i].w_[t] = norm_w(mt);
      clamp(samples_[i].vx_[t], vx_min_, vx_max_);
      clamp(samples_[i].vy_[t], vy_min_, vy_max_);
      clamp(samples_[i].w_[t], w_min_, w_max_);
      // if (pre_vx_[i] * samples_[i].vx_[t] < 0.0)
      //   samples_[i].vx_[t] = pre_vx_[i];
      // pre_vx_[i] = samples_[i].vx_[t];
    }
  }
}

void BrusheeMppiPathTracker::clamp(double& value, double min, double max)
{
  if (value < min)
    value = min;
  else if (value > max)
    value = max;
}

void BrusheeMppiPathTracker::predict_states()  // サンプリングした制御指令を使って状態を予測
{
  for (int i=0; i<num_samples_; i++)
  {
    // path が map のままの場合
    samples_[i].x_[0] = integrated_pose_.pose.position.x;
    samples_[i].y_[0] = integrated_pose_.pose.position.y;
    samples_[i].yaw_[0] = tf::getYaw(integrated_pose_.pose.orientation);

    for (int t=0; t<horizon_-1; t++)
      predict_next_states(samples_[i], t);
  }
  publish_candidate_path();
}

void BrusheeMppiPathTracker::predict_next_states(RobotStates &state, int t)
{
  // ROS_INFO_STREAM("predict_next_states");
  if (is_angler_)
  {
    // 旋回あり
    state.x_[t+1] = state.x_[t] + dt_ * (state.vx_[t] * cos(state.yaw_[t]) - state.vy_[t] * sin(state.yaw_[t]));
    state.y_[t+1] = state.y_[t] + dt_ * (state.vx_[t] * sin(state.yaw_[t]) + state.vy_[t] * cos(state.yaw_[t]));
    state.yaw_[t+1] = state.yaw_[t] + dt_ * state.w_[t];
  }
  else
  {
    // 旋回なし
    state.x_[t+1] = state.x_[t] + dt_ * (state.vx_[t] * cos(state.yaw_[0]) - state.vy_[t] * sin(state.yaw_[0]));
    state.y_[t+1] = state.y_[t] + dt_ * (state.vx_[t] * sin(state.yaw_[0]) + state.vy_[t] * cos(state.yaw_[0]));
  }
}

void BrusheeMppiPathTracker::publish_candidate_path()
{
  candidate_path_msg_.markers.resize(num_samples_);
  for (int i = 0; i < num_samples_; i++)
  {
    candidate_path_msg_.markers[i].header.frame_id = "map";
    candidate_path_msg_.markers[i].header.stamp = ros::Time::now();
    candidate_path_msg_.markers[i].ns = "candidate_path";
    candidate_path_msg_.markers[i].id = i;
    candidate_path_msg_.markers[i].type = visualization_msgs::Marker::LINE_STRIP;
    candidate_path_msg_.markers[i].action = visualization_msgs::Marker::ADD;
    candidate_path_msg_.markers[i].pose.orientation.w = 1.0;
    candidate_path_msg_.markers[i].scale.x = 0.05;
    candidate_path_msg_.markers[i].color.a = 1.0;
    candidate_path_msg_.markers[i].color.r = 0.0;
    candidate_path_msg_.markers[i].color.g = 1.0;
    candidate_path_msg_.markers[i].color.b = 0.0;
    candidate_path_msg_.markers[i].lifetime = ros::Duration();
    candidate_path_msg_.markers[i].points.resize(horizon_);
    for (int t = 0; t < horizon_; t++)
    {
      geometry_msgs::Point p;
      p.x = samples_[i].x_[t];
      p.y = samples_[i].y_[t];
      p.z = 0.0;
      candidate_path_msg_.markers[i].points[t] = p;
    }
  }
  candidate_path_pub_.publish(candidate_path_msg_);
}

void BrusheeMppiPathTracker::calc_weights()
{
  double sum = 0.0;
  for (int i=0; i<num_samples_; i++)
  {
    double cost = calc_cost(samples_[i]);
    weights_[i] = exp(-cost / lambda_);
    sum += weights_[i];
  }
  for (int i=0; i<num_samples_; i++)
    weights_[i] /= sum;
}

void BrusheeMppiPathTracker::calc_ref_path() // 参照経路を計算
{
  int count = 0;
  horizon_ = 20;
  current_index_ = get_current_index();
  double step = vx_ref_ * dt_ / resolution_;
  for (int i=0; i<horizon_; i++)
  {
    int index = current_index_ + step * i;
    if (index < path_.poses.size())
    {
      x_ref_[i] = path_.poses[index].position.x;
      y_ref_[i] = path_.poses[index].position.y;
      yaw_ref_[i] = tf::getYaw(path_.poses[index].orientation);
    }
    else
    {
      x_ref_[i] = path_.poses.back().position.x;
      y_ref_[i] = path_.poses.back().position.y;
      yaw_ref_[i] = tf::getYaw(path_.poses.back().orientation);
      count++;
    }
  }
  horizon_ -= count;
  publish_ref_path();
}

int BrusheeMppiPathTracker::get_current_index()
{
  int index = 0;
  double min_dist = DBL_MAX;
  for (int i=0; i<path_.poses.size()-1; i++)
  {
    double dist = sqrt(pow(integrated_pose_.pose.position.x - path_.poses[i].position.x, 2) + pow(integrated_pose_.pose.position.y - path_.poses[i].position.y, 2));
    if (dist < min_dist)
    {
      min_dist = dist;
      index = i;
    }
  }
  return index;
}

void BrusheeMppiPathTracker::publish_ref_path()
{
  nav_msgs::Path ref_path;
  ref_path.header.frame_id = "map";
  ref_path.header.stamp = ros::Time::now();
  for (int i=0; i<horizon_; i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = x_ref_[i];
    pose.pose.position.y = y_ref_[i];
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_ref_[i]);
    ref_path.poses.push_back(pose);
  }
  ref_path_pub_.publish(ref_path);
}

double BrusheeMppiPathTracker::calc_cost(RobotStates& state)
{
  double cost = 0.0;
  for (int t=0; t<horizon_; t++)
  {
    double dist = calc_min_dist(state, x_ref_, y_ref_);
    double angle_dist = calc_min_angle_dist(state, yaw_ref_);
    double v_cost = abs(sqrt(pow(state.vx_[t], 2) + pow(state.vy_[t], 2)) - vx_ref_);
    double w_cost = pow(state.w_[t] - w_ref_, 2);
    if (is_angler_)
    {
      //  旋回あり
      cost += path_weight_ * dist + vel_weight_ * (v_cost + w_cost) + angle_weight_ * angle_dist;
    }
    else
    {
      //  旋回なし
      cost += path_weight_ * dist + vel_weight_ * v_cost;
    }
  }
  return cost;
}

double BrusheeMppiPathTracker::calc_min_dist(RobotStates& state, std::vector<double>& x_ref, std::vector<double>& y_ref)
{
  double min_dist = DBL_MAX;
  for (int i=0; i<horizon_; i++)
  {
    double dist = sqrt(pow(state.x_[i] - x_ref_[i], 2) + pow(state.y_[i] - y_ref_[i], 2));
    if (dist < min_dist)
      min_dist = dist;
  }
  return min_dist;
}

double BrusheeMppiPathTracker::calc_min_angle_dist(RobotStates& state, std::vector<double>& yaw_ref)
{
  double min_angle_dist = DBL_MAX;
  for (int i=0; i<horizon_; i++)
  {
    double angle_dist = fabs(state.yaw_[i] - yaw_ref_[i]);
    if (angle_dist > M_PI)
      angle_dist = 2 * M_PI - angle_dist;
    if (angle_dist < min_angle_dist)
      min_angle_dist = angle_dist;
  }
  return min_angle_dist;
}

void BrusheeMppiPathTracker::determine_optimal_solution()
{
  for (int t=0; t<horizon_; t++)
  {
    optimal_solution_.vx_[t] = 0.0;
    optimal_solution_.vy_[t] = 0.0;
    optimal_solution_.w_[t] = 0.0;
    for (int i=0; i<num_samples_; i++)
    {
      optimal_solution_.vx_[t] += weights_[i] * samples_[i].vx_[t];
      optimal_solution_.vy_[t] += weights_[i] * samples_[i].vy_[t];
      optimal_solution_.w_[t] += weights_[i] * samples_[i].w_[t];
    }
  }
  publish_optimal_path();
}

void BrusheeMppiPathTracker::publish_optimal_path()
{
  optimal_path_msg_.header.frame_id = "map";
  optimal_path_msg_.header.stamp = ros::Time::now();
  optimal_path_msg_.poses.resize(horizon_-1);
  optimal_solution_.x_[0] = integrated_pose_.pose.position.x;
  optimal_solution_.y_[0] = integrated_pose_.pose.position.y;
  optimal_solution_.yaw_[0] = tf::getYaw(integrated_pose_.pose.orientation);

  for (int t=0; t<horizon_-1; t++)
  {
    predict_next_states(optimal_solution_, t);
    optimal_path_msg_.poses[t].pose.position.x = optimal_solution_.x_[t];
    optimal_path_msg_.poses[t].pose.position.y = optimal_solution_.y_[t];
    optimal_path_msg_.poses[t].pose.orientation = tf::createQuaternionMsgFromYaw(optimal_solution_.yaw_[t]);
  }
  optimal_path_pub_.publish(optimal_path_msg_);
}

void BrusheeMppiPathTracker::publish()
{
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = optimal_solution_.vx_[pub_idx_];
  cmd_vel.linear.y = optimal_solution_.vy_[pub_idx_];
  if (is_angler_)
  {
    // 旋回あり
    cmd_vel.angular.z = optimal_solution_.w_[pub_idx_];
  }
  else
  {
    // 旋回なし
    cmd_vel.angular.z = 0.0;
  }
  ROS_INFO_STREAM_THROTTLE(1.0, "vx: " << cmd_vel.linear.x << ", vy: " << cmd_vel.linear.y << ", v: " << sqrt(pow(cmd_vel.linear.x, 2) + pow(cmd_vel.linear.y, 2)));
  cmd_vel_pub_.publish(cmd_vel);
}