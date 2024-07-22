#include <arbitration_utils/arbitration_utils.h>
#include <arbitration_utils/utils.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>


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
  
  ros::Rate rate(30);
  
  ros::Publisher pubd = nh.advertise<std_msgs::Float32>("/distance_collision_object", 1000);
  ros::Publisher pubr = nh.advertise<std_msgs::Float32>("/reachable_workspace", 1000);
  ros::Publisher pubm = nh.advertise<std_msgs::Float32>("/manipulability_index", 1000);
  ros::Publisher pubc = nh.advertise<std_msgs::Float32>("/closeness_to_target", 1000);
  ros::Publisher puba = nh.advertise<std_msgs::Float32>("/alpha", 1000);
  // ros::Publisher pubvp = nh.advertise<std_msgs::Float32>("/vp_closeness", 1000);

  std::string ee_link;
  nh.getParam("ee_link", ee_link);
  std::string target_pose;
  nh.getParam("target_pose", target_pose);
  // std::string intermediate_point;
  // nh.getParam("intermediate_point", intermediate_point);

  std::cout << "\n";
  ROS_INFO_STREAM("The inizialization part is completed. Now the computational part starts ...");

  while(ros::ok())
  {
    // Manipulability index
    double manipulability_index = au.getCurrentManipulability();
    // Reachable workspace
    double reachable_workspace = au.getReach();
    // distance to object collision
    double distance_to_collision = au.checkWorldCollisionDistance();
    // Distance between ee_link and target_pose
    double closeness_to_target = au.getDistanceFrom(ee_link,target_pose);
    // // Distance between the ee_link and the intermediate_point
    // double vp_clos = au.getDistanceFrom(ee_link, intermediate_point);

    // Here there's a condition in which, if the ws_boundaries param in the config.yaml file is se to false,
    // a predefined value is set for the reachable workspace index
    if(!ws_boundaries)
      reachable_workspace = 0.45;
    
    double alpha = au.computeAlpha(distance_to_collision, reachable_workspace, manipulability_index, closeness_to_target);
    ROS_INFO_STREAM_THROTTLE(2.0, CYAN << "manipulability: " << manipulability_index << 
                                  BLUE << ", reacheable_workspace: " << reachable_workspace << 
                                  GREEN << ", distance_to_collision: " << distance_to_collision << 
                                  YELLOW << ", closeness_to_target: " << closeness_to_target << 
                                  MAGENTA << " ---> alpha: " << alpha);
    
    au.publishAlpha(alpha);
    
    {
      std_msgs::Float32 m;
      m.data = distance_to_collision;
      pubd.publish(m);
    }
    {
      std_msgs::Float32 m;
      m.data = reachable_workspace;
      pubr.publish(m);
    }
    {
      std_msgs::Float32 m;
      m.data = manipulability_index;
      pubm.publish(m);
    }
    {
      std_msgs::Float32 m;
      m.data = alpha;
      puba.publish(m);
    }
    {
      std_msgs::Float32 m;
      m.data = closeness_to_target;
      pubc.publish(m);
    }
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

