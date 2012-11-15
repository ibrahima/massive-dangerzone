#ifndef _SQPP_INTERFACE_ROS_H_
#define _SQPP_INTERFACE_ROS_H_

#include <sqpp_motion_planner/sqpp_planner.h>
#include <sqpp_motion_planner/sqpp_parameters.h>
#include <ros/ros.h>

namespace sqpp_interface_ros
{
/** @class SQPPInterfaceROS */
class SQPPInterfaceROS
{
public:
  SQPPInterfaceROS(const planning_models::KinematicModelConstPtr &kmodel);

  const sqpp::SqppParameters& getParams() const {
    return params_;
  }
  
  bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
             const moveit_msgs::GetMotionPlan::Request &req, 
             const SqppParameters& params,
             moveit_msgs::GetMotionPlan::Response &res) const;

protected:
  
  /** @brief Configure everything using the param server */
  void loadParams(void);
  
  ros::NodeHandle nh_; /// The ROS node handle

  sqpp::SqppParameters params_;  
};

}

#endif
