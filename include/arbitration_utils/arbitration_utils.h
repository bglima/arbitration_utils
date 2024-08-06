#pragma once

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <object_loader_msgs/AddObjects.h>

#include <rosdyn_core/primitives.h>

#include <fl/Headers.h>
#include <tf/transform_listener.h>

class ArbitrationUtils
{
public:
  ros::NodeHandle nh_;
  // To understand what is a shared_ptr, read the following link: https://en.cppreference.com/w/cpp/memory/shared_ptr
  std::shared_ptr< moveit::planning_interface::MoveGroupInterface > move_group_;
  robot_model::RobotModelPtr robot_model_; 
  robot_state::JointModelGroup* joint_model_group_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  
  std::string base_link_;
  std::string tool_link_;
  std::string ee_link_;
  
  rosdyn::ChainPtr chain_bt_;
  rosdyn::ChainPtr chain_bee_;
  
  geometry_msgs::Pose obj_pose_;
  
  std::vector<double> lower_bounds_; 
  std::vector<double> upper_bounds_ ;
  std::string planning_group_;
  std::vector<std::string> joint_names_;
  std::vector<double> pos_;
  std::string alpha_topic_;
  std::string fis_;
  
  ros::ServiceClient  add_obj_ ;
  ros::Publisher alpha_pub_;
  
  double max_fl_;
  double min_fl_;
  
  fl::Engine*         engine_;
  fl::InputVariable*  manipulability_ ;
  fl::InputVariable*  distance_ ;
  fl::InputVariable*  reach_ ;
  fl::InputVariable*  endpoint_ ;
  fl::OutputVariable* alpha_ ;
  
  tf::TransformListener listener_;
  
  ArbitrationUtils(ros::NodeHandle nh);
  double getManipulability(const std::vector<double> joints);
  double getCurrentManipulability();
  double getReach();
  double getDistanceFrom(const std::string& base, const std::string& target);
  void getPlanningScene( ros::NodeHandle& nh, planning_scene::PlanningScenePtr& ret );
  void addObj();
  double checkWorldCollisionDistance();
  double computeAlpha(const double& dist, const double& reach, const double& man, const double& clos);
  void publishAlpha(const double& alpha);
  
  void vecToPose(const std::vector<double>& pose ,geometry_msgs::Pose& gpose); 
};
