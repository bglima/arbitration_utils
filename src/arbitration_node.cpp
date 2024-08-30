#include <arbitration_utils/arbitration_utils.h>
#include <arbitration_utils/utils.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

bool human_lead_enabled = false;

void human_leadCallback(const std_msgs::Bool::ConstPtr& msg)
{
     human_lead_enabled = msg->data;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "arbitration_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  
  // Initialization part
  ArbitrationUtils au(nh);
  
  // Check if the parameter in the config.yaml file is set
  bool add_obj;

  if(!nh.getParam("add_object", add_obj))
  {
    ROS_WARN_STREAM(nh.getNamespace() << " /no object found . default false");
    add_obj = false;
  }

  // Check if the parameter in the config.yaml file is set
  bool ws_boundaries;

  if(!nh.getParam("ws_boundaries", ws_boundaries))
  {
    ROS_WARN_STREAM(nh.getNamespace() << " /no object found. default false");
    ws_boundaries = true;
  }

  if (add_obj)
    au.addObj();

  ros::Duration(0.1).sleep();
  
  double rate_hz = 30;
  double dt = 1/rate_hz;
  ros::Rate rate(rate_hz);
  
  ros::Publisher pubd = nh.advertise<std_msgs::Float32>("/distance_collision_object", 1);
  ros::Publisher pubr = nh.advertise<std_msgs::Float32>("/reachable_workspace", 1);
  ros::Publisher pubm = nh.advertise<std_msgs::Float32>("/manipulability_index", 1);
  ros::Publisher pubc = nh.advertise<std_msgs::Float32>("/closeness_to_target", 1);
  ros::Publisher puba = nh.advertise<std_msgs::Float32>("/alpha", 1);
    // ros::Publisher pubvp = nh.advertise<std_msgs::Float32>("/vp_closeness", 1000);

  ros::Subscriber enable_human_leading_sub = nh.subscribe<std_msgs::Bool>("/human_lead_enabled", 1, human_leadCallback);  

  std::string ee_link;
  nh.getParam("ee_link", ee_link);
  std::string target_pose;
  nh.getParam("target_pose", target_pose);
  // std::string intermediate_point;
  // nh.getParam("intermediate_point", intermediate_point);

  std::cout << "\n";
  ROS_INFO_STREAM("The inizialization part is completed. Now the computational part starts ...");

  std_msgs::Float32 float_msg;
  
  ros::Time time_now;

  while(ros::ok())
  {
    time_now = ros::Time::now();
    // Manipulability index
    double manipulability_index = au.getCurrentManipulability();
    ROS_WARN_THROTTLE(2.0, "getCurrentManipulability took: %.4f seconds.", (ros::Time::now() - time_now).toSec() );
    time_now = ros::Time::now();
    // Reachable workspace
    double reachable_workspace = au.getReach();
    ROS_WARN_THROTTLE(2.0, "reachable_workspace took: %.4f seconds.", (ros::Time::now() - time_now).toSec() );
    time_now = ros::Time::now();
        // distance to object collision
    double distance_to_collision = au.checkWorldCollisionDistance();
    ROS_WARN_THROTTLE(2.0, "distance_to_collision took: %.4f seconds.", (ros::Time::now() - time_now).toSec() );
    time_now = ros::Time::now();
    
    // Distance between ee_link and target_pose
    double closeness_to_target = au.getDistanceFrom(ee_link,target_pose);
    ROS_WARN_THROTTLE(2.0, "closeness_to_target took: %.4f seconds.", (ros::Time::now() - time_now).toSec() );
    time_now = ros::Time::now();
    
    // // Distance between the ee_link and the intermediate_point
    // double vp_clos = au.getDistanceFrom(ee_link, intermediate_point);

    // Here there's a condition in which, if the ws_boundaries param in the config.yaml file is set to false,
    // a predefined value is set for the reachable workspace index
    if(!ws_boundaries)
      reachable_workspace = 0.45;
    
    //double alpha;
    //if (human_lead_enabled == false)
    //{
    //  alpha = au.computeAlpha(distance_to_collision, reachable_workspace, manipulability_index, closeness_to_target);
   // }
   // else
    //{
    //  alpha = 0.999;
   // }

    ROS_INFO_STREAM_THROTTLE(2.0, CYAN << "manipulability: " << manipulability_index << 
                                  BLUE << ", reacheable_workspace: " << reachable_workspace << 
                                  GREEN << ", distance_to_collision: " << distance_to_collision << 
                                  YELLOW << ", closeness_to_target: " << closeness_to_target);
    
    ROS_WARN_THROTTLE(2.0, "computeAlpha took: %.4f seconds.", (ros::Time::now() - time_now).toSec() );
    time_now = ros::Time::now();

    //float_msg.data = alpha;
    //puba.publish(float_msg);

    float_msg.data = distance_to_collision;
    pubd.publish(float_msg);

    float_msg.data = reachable_workspace;
    pubr.publish(float_msg);

    float_msg.data = manipulability_index;
    pubm.publish(float_msg);

    float_msg.data = closeness_to_target;
    pubc.publish(float_msg);
    // {
    //   std_msgs::Float32 m;
    //   m.data = vp_clos;
    //   pubvp.publish(m);
    // }
    
    rate.sleep();
  }
  ros::waitForShutdown();
  
  return 0;
  
}

