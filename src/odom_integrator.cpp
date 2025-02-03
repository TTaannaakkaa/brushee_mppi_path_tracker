// copyright 2025 amsl

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

#include <optional>

class OdomIntegrator
{
public:
  OdomIntegrator(void):private_nh_("~")
  {
    amcl_pose_sub_ = nh_.subscribe("/amcl_pose", 1, &OdomIntegrator::amcl_pose_callback, this);
    odom_sub_ = nh_.subscribe("/odom", 1, &OdomIntegrator::odom_callback, this);

    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/integrated_pose", 1);
  }
private:

  void amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    amcl_pose_ = *msg;
    is_amcl_pose_ = true;
  }

  void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    if (is_amcl_pose_ && last_odom_.has_value())
    {
      integrated_pose_.header.frame_id = "map";
      integrated_pose_.header.stamp = ros::Time::now();
      integrated_pose_.pose.position.x = amcl_pose_.pose.pose.position.x;
      integrated_pose_.pose.position.y = amcl_pose_.pose.pose.position.y;
      integrated_pose_.pose.orientation = amcl_pose_.pose.pose.orientation;
      yaw_ = tf2::getYaw(integrated_pose_.pose.orientation);
      pose_pub_.publish(integrated_pose_);
      is_amcl_pose_ = false;
    }
    else if (last_odom_.has_value())
    {
      const double dt = (msg->header.stamp - last_odom_->header.stamp).toSec();
      integrated_pose_.pose.position.x += msg->twist.twist.linear.x * cos(yaw_) * dt - msg->twist.twist.linear.y * sin(yaw_) * dt;
      integrated_pose_.pose.position.y += msg->twist.twist.linear.x * sin(yaw_) * dt + msg->twist.twist.linear.y * cos(yaw_) * dt;
      yaw_ += msg->twist.twist.angular.z * dt;
      publish_integrated_pose(msg->header.stamp);
    }
    last_odom_ = *msg;
  }

  void publish_integrated_pose(const ros::Time &stamp)
  {
    integrated_pose_.header.frame_id = "map";
    integrated_pose_.header.stamp = ros::Time::now();
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    tf2::convert(q, integrated_pose_.pose.orientation);
    pose_pub_.publish(integrated_pose_);

    geometry_msgs::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = integrated_pose_.pose.position.x;
    transform.transform.translation.y = integrated_pose_.pose.position.y;
    transform.transform.rotation = integrated_pose_.pose.orientation;
    static tf2_ros::TransformBroadcaster br;
    br.sendTransform(transform);
  }

  double yaw_ = 0.0;
  bool is_amcl_pose_ = false;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber amcl_pose_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher pose_pub_;

  geometry_msgs::PoseStamped integrated_pose_;
  geometry_msgs::PoseWithCovarianceStamped amcl_pose_;
  std::optional<nav_msgs::Odometry> last_odom_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_integrator");
  OdomIntegrator odom_integrator;
  ros::spin();
  return 0;
}