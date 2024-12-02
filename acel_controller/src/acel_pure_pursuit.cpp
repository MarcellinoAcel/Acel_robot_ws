
#include <algorithm>
#include <string>
#include <memory>
#include <cstdio>
#include <stdio.h>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "acel_controller/acel_pure_pursuit.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "acel_controller/convertion.hpp"
#include "acel_controller/pid.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "icecream.hpp"
using namespace std;

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using std::abs;
using std::hypot;
using std::max;
using std::min;
using std::placeholders::_1;
PID omni;
Convertion convert;

namespace acel_pure_pursuit
{
  template <typename Iter, typename Getter>
  Iter min_by(Iter begin, Iter end, Getter getCompareVal)
  {
    if (begin == end)
    {
      return end;
    }
    auto lowest = getCompareVal(*begin);
    Iter lowest_it = begin;
    for (Iter it = ++begin; it != end; ++it)
    {
      auto comp = getCompareVal(*it);
      if (comp < lowest)
      {
        lowest = comp;
        lowest_it = it;
      }
    }
    return lowest_it;
  }

  void Acel_pure_pursuit::configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
      std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    node_ = parent;

    auto node = node_.lock();

    costmap_ros_ = costmap_ros;
    tf_ = tf;
    plugin_name_ = name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    declare_parameter_if_not_declared(
        node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(5.0));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".lookahead_dist",
        rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(3.0));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".kp", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".ki", rclcpp::ParameterValue(0.0));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".kd", rclcpp::ParameterValue(0.0));

    node->get_parameter(plugin_name_ + ".kp", parameters.kp);
    node->get_parameter(plugin_name_ + ".ki", parameters.ki);
    node->get_parameter(plugin_name_ + ".kd", parameters.kd);
    node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
    node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
    node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
    double transform_tolerance;
    node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
    transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);

    global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
    sub_amcl_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "amcl_pose", 10, std::bind(&Acel_pure_pursuit::robot_pose, this, std::placeholders::_1));
    pub_goal = node->create_publisher<geometry_msgs::msg::Pose2D>("checking_goal", 1);
  }

  void Acel_pure_pursuit::cleanup()
  {
    RCLCPP_INFO(
        logger_,
        "Cleaning up controller: %s of type pure_pursuit_controller::Acel_pure_pursuit",
        plugin_name_.c_str());
    global_pub_.reset();
  }

  void Acel_pure_pursuit::activate()
  {
    RCLCPP_INFO(
        logger_,
        "Activating controller: %s of type pure_pursuit_controller::Acel_pure_pursuit\"  %s",
        plugin_name_.c_str(), plugin_name_.c_str());
    global_pub_->on_activate();
  }

  void Acel_pure_pursuit::deactivate()
  {
    RCLCPP_INFO(
        logger_,
        "Dectivating controller: %s of type pure_pursuit_controller::Acel_pure_pursuit\"  %s",
        plugin_name_.c_str(), plugin_name_.c_str());
    global_pub_->on_deactivate();
  }

  void Acel_pure_pursuit::setSpeedLimit(const double &speed_limit, const bool &percentage)
  {
    (void)speed_limit;
    (void)percentage;
  }

  geometry_msgs::msg::TwistStamped Acel_pure_pursuit::computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &pose,
      const geometry_msgs::msg::Twist &velocity,
      nav2_core::GoalChecker *goal_checker)
  {
    auto amcl_pose_ = current_pose_;
    auto goal_pose_msg = geometry_msgs::msg::Pose2D();
    (void)velocity;
    (void)goal_checker;

    auto transformed_plan = transformGlobalPlan(pose);
    auto transformed_goal_pose = transformGlobalPoseToLocal(pose);
    auto global_goal_pose = transformed_goal_pose;

    // Find the first pose which is at a distance greater than the specified lookahed distance
    auto goal_pose_it = std::find_if(
        transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto &ps)
        { return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist_; });

    // If the last pose is still within lookahed distance, take the last pose
    if (goal_pose_it == transformed_plan.poses.end())
    {
      goal_pose_it = std::prev(transformed_plan.poses.end());
    }
    // auto goal_pose = goal_pose_it->pose;
    Convertion::Quaternion goal_q = {
        global_goal_pose.pose.orientation.w,
        global_goal_pose.pose.orientation.x,
        global_goal_pose.pose.orientation.y,
        global_goal_pose.pose.orientation.z,
    };

    double goal_yaw, goal_pitch, goal_roll;
    convert.quat_to_eular(goal_q, goal_yaw, goal_pitch, goal_roll);

    Convertion::Quaternion robot_q = {
        amcl_pose_.pose.pose.orientation.w,
        amcl_pose_.pose.pose.orientation.x,
        amcl_pose_.pose.pose.orientation.y,
        amcl_pose_.pose.pose.orientation.z,
    };
    double robot_yaw, robot_pitch, robot_roll;
    convert.quat_to_eular(robot_q, robot_yaw, robot_pitch, robot_roll);
    error.x = global_goal_pose.pose.position.x - amcl_pose_.pose.pose.position.x;
    error.y = global_goal_pose.pose.position.y - amcl_pose_.pose.pose.position.y;
    error.theta = goal_yaw - robot_yaw;
    error.distance = hypot(error.x, error.y);
    error.angle = atan2(error.y, error.x);
    // IC(goal_pose.position.x, goal_pose.position.y);

    omni.setBaseParam(parameters.kp, parameters.kp, parameters.kp);
    omni.setHeadingParam(parameters.kp, parameters.kp, parameters.kp);

    controlled.distance = omni.control_base(error.distance, desired_linear_vel_, 0);
    controlled.angle = omni.control_base(error.angle, max_angular_vel_, 1);

    goal_pose_msg.x = global_goal_pose.pose.position.x;
    goal_pose_msg.y = global_goal_pose.pose.position.y;
    goal_pose_msg.theta = goal_yaw;

    pub_goal->publish(goal_pose_msg);

    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = pose.header.frame_id;
    cmd_vel.header.stamp = clock_->now();
    // -----------------------------------------------------------
    cmd_vel.twist.linear.x = controlled.distance * cos(error.angle);
    cmd_vel.twist.linear.y = controlled.distance * sin(error.angle);
    cmd_vel.twist.linear.z = 0;

    return cmd_vel;
  }

  void Acel_pure_pursuit::setPlan(const nav_msgs::msg::Path &path)
  {
    global_pub_->publish(path);
    global_plan_ = path;
  }

  geometry_msgs::msg::PoseStamped Acel_pure_pursuit::transformGlobalPoseToLocal(
      const geometry_msgs::msg::PoseStamped &pose)
  {
    if (global_plan_.poses.empty())
    {
      throw nav2_core::PlannerException("Received plan with zero length");
    }

    // let's get the pose of the robot in the frame of the plan
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!transformPose(
            tf_, global_plan_.header.frame_id, pose,
            robot_pose, transform_tolerance_))
    {
      throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
    }

    // We'll discard points on the plan that are outside the local costmap
    nav2_costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
    double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
                            costmap->getResolution() / 2.0;

    // First find the closest pose on the path to the robot
    auto transformation_begin =
        min_by(
            global_plan_.poses.begin(), global_plan_.poses.end(),
            [&robot_pose](const geometry_msgs::msg::PoseStamped &ps)
            {
              return euclidean_distance(robot_pose, ps);
            });

    // From the closest point, look for the first point that's further then dist_threshold from the
    // robot. These points are definitely outside of the costmap so we won't transform them.
    auto transformation_end = std::find_if(
        transformation_begin, end(global_plan_.poses),
        [&](const auto &global_plan_pose)
        {
          return euclidean_distance(robot_pose, global_plan_pose) > dist_threshold;
        });

    // Helper function for the transform below. Transforms a PoseStamped from global frame to local
    geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
    auto transformGlobalPoseToLocal = [&](const auto &global_plan_pose)
    {
      // We took a copy of the pose, let's lookup the transform at the current time
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      transformPose(
          tf_, costmap_ros_->getBaseFrameID(),
          stamped_pose, transformed_pose, transform_tolerance_);
      // RCLCPP_INFO(logger_, "Transformed pose: global (x=%.2f, y=%.2f) -> local (x=%.2f, y=%.2f)",
      //             global_plan_pose.pose.position.x, global_plan_pose.pose.position.y,
      //             transformed_pose.pose.position.x, transformed_pose.pose.position.y);

      return transformed_pose;
    };

    // RCLCPP_INFO(logger_, "local (x=%.2f, y=%.2f)",
    //             transformed_pose.pose.position.x, transformed_pose.pose.position.y);

    return stamped_pose;
  }

  nav_msgs::msg::Path Acel_pure_pursuit::transformGlobalPlan(
      const geometry_msgs::msg::PoseStamped &pose)
  {
    // Original mplementation taken fron nav2_dwb_controller

    if (global_plan_.poses.empty())
    {
      throw nav2_core::PlannerException("Received plan with zero length");
    }

    // let's get the pose of the robot in the frame of the plan
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!transformPose(
            tf_, global_plan_.header.frame_id, pose,
            robot_pose, transform_tolerance_))
    {
      throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
    }

    // RCLCPP_INFO(logger_, "Robot position in global frame: x=%.2f, y=%.2f",
    //             robot_pose.pose.position.x, robot_pose.pose.position.y);

    // We'll discard points on the plan that are outside the local costmap
    nav2_costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
    double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
                            costmap->getResolution() / 2.0;

    // First find the closest pose on the path to the robot
    auto transformation_begin =
        min_by(
            global_plan_.poses.begin(), global_plan_.poses.end(),
            [&robot_pose](const geometry_msgs::msg::PoseStamped &ps)
            {
              return euclidean_distance(robot_pose, ps);
            });
    // RCLCPP_INFO(logger_, "Closest pose to robot: x=%.2f, y=%.2f",
    //             transformation_begin->pose.position.x, transformation_begin->pose.position.y);

    // From the closest point, look for the first point that's further then dist_threshold from the
    // robot. These points are definitely outside of the costmap so we won't transform them.
    auto transformation_end = std::find_if(
        transformation_begin, end(global_plan_.poses),
        [&](const auto &global_plan_pose)
        {
          return euclidean_distance(robot_pose, global_plan_pose) > dist_threshold;
        });

    // Helper function for the transform below. Transforms a PoseStamped from global frame to local
    auto transformGlobalPoseToLocal = [&](const auto &global_plan_pose)
    {
      // We took a copy of the pose, let's lookup the transform at the current time
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      transformPose(
          tf_, costmap_ros_->getBaseFrameID(),
          stamped_pose, transformed_pose, transform_tolerance_);
      RCLCPP_INFO(logger_, "Transformed pose: global (x=%.2f, y=%.2f) -> local (x=%.2f, y=%.2f)",
                  global_plan_pose.pose.position.x, global_plan_pose.pose.position.y,
                  transformed_pose.pose.position.x, transformed_pose.pose.position.y);

      return transformed_pose;
    };

    // Transform the near part of the global plan into the robot's frame of reference.
    nav_msgs::msg::Path transformed_plan;
    std::transform(
        transformation_begin, transformation_end,
        std::back_inserter(transformed_plan.poses),
        transformGlobalPoseToLocal);
    transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
    transformed_plan.header.stamp = pose.header.stamp;

    // Remove the portion of the global plan that we've already passed so we don't
    // process it on the next iteration (this is called path pruning)
    global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
    global_pub_->publish(transformed_plan);

    if (transformed_plan.poses.empty())
    {
      throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
    }

    return transformed_plan;
  }

  bool Acel_pure_pursuit::transformPose(
      const std::shared_ptr<tf2_ros::Buffer> tf,
      const std::string frame,
      const geometry_msgs::msg::PoseStamped &in_pose,
      geometry_msgs::msg::PoseStamped &out_pose,
      const rclcpp::Duration &transform_tolerance) const
  {
    if (in_pose.header.frame_id == frame)
    {
      out_pose = in_pose;
      return true;
    }

    try
    {
      tf->transform(in_pose, out_pose, frame);
      return true;
    }
    catch (tf2::ExtrapolationException &ex)
    {
      auto transform = tf->lookupTransform(
          frame,
          in_pose.header.frame_id,
          tf2::TimePointZero);
      if (
          (rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) >
          transform_tolerance)
      {
        RCLCPP_ERROR(
            rclcpp::get_logger("tf_help"),
            "Transform data too old when converting from %s to %s",
            in_pose.header.frame_id.c_str(),
            frame.c_str());
        RCLCPP_ERROR(
            rclcpp::get_logger("tf_help"),
            "Data time: %ds %uns, Transform time: %ds %uns",
            in_pose.header.stamp.sec,
            in_pose.header.stamp.nanosec,
            transform.header.stamp.sec,
            transform.header.stamp.nanosec);
        return false;
      }
      else
      {
        tf2::doTransform(in_pose, out_pose, transform);
        return true;
      }
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("tf_help"),
          "Exception in transformPose: %s",
          ex.what());
      return false;
    }
    return false;
  }

} // namespace acel_pure_pursuit

PLUGINLIB_EXPORT_CLASS(acel_pure_pursuit::Acel_pure_pursuit, nav2_core::Controller)
