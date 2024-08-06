#include <arbitration_utils/arbitration_utils.h>
#include <arbitration_utils/utils.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <ros/console.h>

ArbitrationUtils::ArbitrationUtils(ros::NodeHandle nh):nh_(nh)
{
  if ( !nh_.getParam ( "planning_group", planning_group_) )
  {
    ROS_ERROR_STREAM (nh_.getNamespace() << " /planning_group not set. return");
    return;
  }
  if ( !nh_.getParam ( "base_link", base_link_) )
  {
    ROS_ERROR_STREAM (nh_.getNamespace() << " /base_link not set. return");
    return;
  }

  if ( !nh_.getParam ( "tool_link", tool_link_) )
  {
    ROS_ERROR_STREAM (nh_.getNamespace() << " /tool_link not set. return");
    return;
  }

  if ( !nh_.getParam ( "ee_link", ee_link_) )
  {
    ROS_ERROR_STREAM (nh_.getNamespace() << " /ee_link not set. return");
    return;
  }

  if ( !nh_.getParam ( "joint_names", joint_names_) )
  {
    ROS_ERROR_STREAM (nh_.getNamespace() << " /joint_names not set. return");
    return;
  }

  if ( !nh_.getParam ( "object_geometries/pose", pos_) )
  {
    ROS_ERROR_STREAM (nh_.getNamespace() << " /object_geometries/pose not set. return");
    return;
  }

  vecToPose(pos_,obj_pose_);
  
  if ( !nh_.getParam ( "max_fl", max_fl_) )
  {
    ROS_WARN_STREAM (nh_.getNamespace() << " /max_fl not set. default 0.99");
    max_fl_ = 0.99;
  }
  if ( !nh_.getParam ( "min_fl", min_fl_) )
  {
    ROS_WARN_STREAM (nh_.getNamespace() << " /min_fl not set. default 0.01");
    min_fl_ = 0.01;
  }

  move_group_.reset(new moveit::planning_interface::MoveGroupInterface(planning_group_));
  robot_model_ = robot_model_loader::RobotModelLoader("robot_description").getModel();
  joint_model_group_ = robot_model_->getJointModelGroup(planning_group_);
  
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));
  
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader_)); 
  planning_scene_monitor_->startSceneMonitor();
  planning_scene_monitor_->startWorldGeometryMonitor();

  // Update planning scene monitor with the current Planning Scene state
  // https://answers.ros.org/question/356183/how-to-use-the-planning-scene-monitor-in-c/
  bool success = planning_scene_monitor_->requestPlanningSceneState("/get_planning_scene");
  if (!success)
  {
    ROS_ERROR("Could not update planning scene monitor with the current planning scene state.");
  }
  
  add_obj_ = nh_.serviceClient<object_loader_msgs::AddObjects> ( "add_object_to_scene" );
  ROS_INFO_STREAM("waiting for service: "<< add_obj_.getService());
  add_obj_.waitForExistence();
  
  if ( !nh_.getParam ( "alpha_topic", alpha_topic_) )
  {
    ROS_ERROR_STREAM (nh_.getNamespace() << " /alpha_topic not set. default "<< nh_.getNamespace()<<"/alpha");
    alpha_topic_ = nh_.getNamespace()+"/alpha";
  }
  alpha_pub_ = nh_.advertise< std_msgs::Float32 >(alpha_topic_, 1000);
  
  urdf::Model urdf_model;
  if (!urdf_model.initParam("robot_description"))
  {
    ROS_ERROR("Urdf robot_description '%s' does not exist",(nh_.getNamespace()+"/robot_description").c_str());
  }
  Eigen::Vector3d gravity;
  gravity << 0, 0, -9.806;
  chain_bt_  = rosdyn::createChain(urdf_model, base_link_, tool_link_, gravity);
  chain_bee_ = rosdyn::createChain(urdf_model, base_link_, ee_link_, gravity);

  for (auto j:joint_names_)
  {
    joint_limits_interface::JointLimits limits;
    urdf::JointConstSharedPtr urdf_joint = urdf_model.getJoint(j);
    if( ! getJointLimits(urdf_joint, limits) )
    {
      ROS_ERROR_STREAM("error in reading joint "<< j <<" limits");
    }
    upper_bounds_.push_back ( limits.max_position );
    lower_bounds_.push_back ( limits.min_position );
  }
  
  std::vector<double> cj = move_group_->getCurrentJointValues();
  
  for (size_t i=0;i<joint_names_.size();i++)
    ROS_INFO_STREAM(BLUE<<"joint_"<< joint_names_.at(i) <<" current:" << cj.at(i) <<" - upper joint: "<< upper_bounds_.at(i) <<" - lower joint: "<<lower_bounds_.at(i));
  
  if ( !nh_.getParam ( "alpha_fis", fis_) )
  {
    ROS_ERROR_STREAM (nh_.getNamespace() << " /alpha_fis not set. return");
    return;
  }
  std::string path = ros::package::getPath("arbitration_utils");
  path += "/config/";
  path += fis_;
  
  ROS_INFO_STREAM("recovering path: " << path);
  
  engine_ = fl::FisImporter().fromFile(path);
  std::string status;
  if (not engine_->isReady(&status))
    ROS_ERROR_STREAM("engine is not ready");

  manipulability_  = engine_->getInputVariable("manipulability");
  distance_        = engine_->getInputVariable("distance");
  reach_           = engine_->getInputVariable("reach");
  endpoint_        = engine_->getInputVariable("endpoint");
  alpha_           = engine_->getOutputVariable("alpha");

}


double ArbitrationUtils::getManipulability(const std::vector<double> joints)
{
  Eigen::VectorXd j;
  
  j.resize(joints.size());

  // std::cout << "the dimension of the vector j is: \n" << j.size() << "\n";

  for (int i=0; i < joints.size(); i++)
  {
    j(i) = joints[i];
  }

  // std::cout << "the actual joints values are: \n" << j << "\n";

  Eigen::MatrixXd  J_b = chain_bt_->getJacobian(j);

  // std::cout << "the jacobian matrix, evaluated in this joint values, is: \n" << J_b << "\n";

  std::vector<double> prod;
  
  // This for loop uses the "auto" to print each element of the joints vector
  for (auto j:joints)
    ROS_DEBUG_STREAM("j: "<<j);
  
  // Computation of the Penalty factor P and then the manipulability index mu
  for (int i=0; i<joints.size(); i++)
  {
    double num = (joints[i] - lower_bounds_[i])*(upper_bounds_[i] - joints[i]); 
    double den = upper_bounds_[i] - lower_bounds_[i];
    prod.push_back( num / pow(den,2.0));
  }

  // std::cout << "the exponential terms in the penalty factor, without k, are: \n";
  // for(int i =0; i< joints.size(); i++)
  //   std::cout << prod[i] << "\n";

  // In this case, it is considered the minimum value of the exponential term in the formula
  double min_prod = *std::min_element(prod.begin(),prod.end());
  double k = 100;
  // Penalty factor P
  double penalty = 1 - exp(-k * min_prod);
  
  ROS_DEBUG_STREAM("mu: " << (std::sqrt((J_b * J_b.transpose()).determinant())));
  ROS_DEBUG_STREAM("penalty factor: " << penalty);

  // std::cout << "the minimum value between the exponential terms is: \n" << min_prod << "\n";
  // std::cout << "the value of k is: \n" << k << "\n";
  // std::cout << "the penalty value is: " << penalty << "\n";  
  // std::cout << "mu is equal to: " << std::sqrt((J_b * J_b.transpose()).determinant()) << "\n";
  // std::cout << "the manipulability index is: " << (std::sqrt((J_b * J_b.transpose()).determinant())) * penalty << "\n";  

  return (std::sqrt((J_b * J_b.transpose()).determinant())) * penalty;
}


double ArbitrationUtils::getCurrentManipulability()
{
  return getManipulability(move_group_->getCurrentJointValues());
}


double ArbitrationUtils::getReach()
{
  std::vector<double> cjv = move_group_->getCurrentJointValues();

  // To understand how Eigen::Map, go to the followng link: https://eigen.tuxfamily.org/dox/classEigen_1_1Map.html
  Eigen::VectorXd vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(cjv.data(), cjv.size());
  // In here, you print a vector with the current joint values
  // std::cout << "the variable vec is: \n" << vec << "\n";
  // In here, you compute basically the forward kinematics to pass from joint values to task space values
  Eigen::Affine3d T_be = chain_bee_->getTransformation(vec);
  // In here, yout print only the task space translation from the base frame to the ee_frame (defined as panda_hand_tcp)
  // std::cout << "the variable T_be is: \n" << T_be.translation() << "\n"; 
  // In Here, the norm of the translation vector from base frame to ee_frame is computed to get the reachable workspace
  double reach = T_be.translation().norm();
  
  return reach; 
}

double ArbitrationUtils::getDistanceFrom(const std::string& base, const std::string& target)
{
  tf::StampedTransform transform;
  // tf::StampedTransform tt; // used for the intermediate point that, in our case, is not considered
  double distance = 0.5;

  try
  {
    listener_.waitForTransform(base, target, ros::Time::now(), ros::Duration(5.0));
    listener_.lookupTransform(base, target, ros::Time(0), transform);
    // listener_.lookupTransform(base, target, ros::Time(0), tt);
    
    Eigen::Vector3d v;
    // tf::vectorTFToEigen converts a tf Vector3 into an Eigen Vector3d
    tf::vectorTFToEigen(transform.getOrigin(),v);
    distance = v.norm();

    if (distance < 0.001)
      ROS_INFO_STREAM_THROTTLE(1,GREEN << "Target pose reached! Current distance < 1mm: " << distance);
    else
      ROS_INFO_STREAM_THROTTLE(1,RED << "Distance between ee_link and target pose > 1mm: " << distance);
    
    // Eigen::Vector3d vt;
    // tf::vectorTFToEigen(tt.getOrigin(),vt);
    // double tttpdist = vt.norm();
    // if (tttpdist < 0.001)
    //   ROS_INFO_STREAM_THROTTLE(1,GREEN<<"distance between tip tbt ok <1mm: " << tttpdist );
    // else
    //   ROS_INFO_STREAM_THROTTLE(1,RED<<"distance between tip tbt > 1mm: : " << tttpdist );
    
    // ros::Publisher pub = nh_.advertise<std_msgs::Float32>("/distance_to_goal",1);
    
    // std_msgs::Float32 msg;
    // msg.data = distance;
    // pub.publish(msg);
  }
  catch (tf::TransformException &ex)
  {
    ROS_WARN_STREAM_THROTTLE(2.0,"transform not yet recovered, skipping");
    ROS_ERROR("%s",ex.what());
  }
  
  return distance;
  
}
void ArbitrationUtils::addObj()
{
  ROS_DEBUG_STREAM("adding object");
  
  // object generation through a service in the object_loader package
  object_loader_msgs::AddObjects srv;  
  {
    object_loader_msgs::Object obj;
    
    obj.object_type="box";
    
    obj.pose.pose = obj_pose_;
    obj.pose.header.frame_id = base_link_;
    srv.request.objects.push_back(obj);
  }
  add_obj_.call(srv);
}


double ArbitrationUtils::checkWorldCollisionDistance()
{
  // Receive a pointer to the planning scene in ReadOnly mode to read the current state.
  bool success = planning_scene_monitor_->requestPlanningSceneState("/get_planning_scene");
  if (!success)
  {
    ROS_ERROR("Could not update planning scene monitor with the current planning scene state.");
  }

  // The ReadOnly mode is so as not to interfeer with other publishers
  planning_scene_monitor::LockedPlanningSceneRO locked_planning_scene_read_only = planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_); 

  // Get the state at which the robot is assumed to be.
  robot_state::RobotState robot_state = locked_planning_scene_read_only->getCurrentState();

  // Get the current allowed collision detection matrix
  const collision_detection::AllowedCollisionMatrix *acm = &locked_planning_scene_read_only->getAllowedCollisionMatrix();
  
  // Calculate the distance based on the allowed collision matrix
  double dist = locked_planning_scene_read_only->distanceToCollision(robot_state, *acm);

  // If distance is too close, inform a collision
  if (dist <= 0.0001)
  {
    ROS_INFO_STREAM(RED<<"careful ! robot in collision");
    dist = 0.0001;
  }
  
  return dist;
}


double ArbitrationUtils::computeAlpha(const double& dist, const double& reach, const double& man, const double& clos)
{
  manipulability_->setValue(man);
  distance_->setValue(dist);
  reach_->setValue(reach);
  endpoint_->setValue(clos);
  
  engine_->process();
  
  double out = alpha_->getValue();
  
  if ( isnan(out) )
  {
    ROS_INFO_STREAM_THROTTLE(5.0,"setting alpha to 0.999");
    out = max_fl_;
  }
  
  double ret = (out - min_fl_)/(max_fl_ - min_fl_);
  
  if (ret == 1)
    ret = 0.999;
  
  ROS_DEBUG_STREAM_THROTTLE(2.0,"given manipulability: " << manipulability_->getValue() << 
                                ", and reach " << reach_->getValue() << 
                                ", and distance " << distance_->getValue() << 
                                ", and endpoint closeness " << endpoint_->getValue() << 
                                ", fl returns alpha = " << ret);
  
  return ret;
}


void ArbitrationUtils::publishAlpha(const double& alpha)
{  
  std_msgs::Float32 alpha_msg;
  
  alpha_msg.data = alpha;
  
  alpha_pub_.publish(alpha_msg);
  
  return ;
}

// This function convert a Pose defined in a std::vector (exploting TF class reference) into a geometry_msg::Pose
void ArbitrationUtils::vecToPose(const std::vector<double>& pose ,geometry_msgs::Pose& gpose)
  {
    // A representation of pose (A position and orientation)
    tf::Pose transform;
    
    // tf::Vector3 can be used to represent 3D points and vectors. 
    tf::Vector3 v = tf::Vector3(pose.at(0),pose.at(1),pose.at(2));
    // The Quaternion implements quaternion to perform linear algebra rotations in combination with Matrix3x3, Vector3 
    // and Transform
    tf::Quaternion q;
    
    // The assignment is different if in the scene_objects.yaml we define RPY form or quaternion form.
    if(pose.size() == 6)
    q = tf::createQuaternionFromRPY(pose.at(3),pose.at(4),pose.at(5)); 
    else
    // In this case, the sequence is x, y, z and w
    q = tf::Quaternion(pose.at(4),pose.at(5),pose.at(6),pose.at(3)); 
    
    // Set the translational element
    transform.setOrigin(v);
    // Set the rotational element by Quaternion
    transform.setRotation(q); 
    
    // Convert a Pose (in a TF configuration) into a Pose Msg (geometry_msg::Pose)
    tf::poseTFToMsg(transform, gpose);
  }








