#include <sqpp_interface_ros/sqpp_interface_ros.h>

namespace sqpp_interface_ros
{

SQPPInterfaceROS::SQPPInterfaceROS(const planning_models::KinematicModelConstPtr& kmodel) :
  SqpPlanner(kmodel), nh_("~") 
{
  loadParams();
}

void SQPPInterfaceROS::loadParams(void) {
  // nh_.param("planning_time_limit", params_.planning_time_limit_, 6.0);
  // nh_.param("max_iterations", params_.max_iterations_, 50);
  // nh_.param("max_iterations_after_collision_free", params_.max_iterations_after_collision_free_, 5);
  // nh_.param("smoothness_cost_weight", params_.smoothness_cost_weight_, 0.1);
  // nh_.param("obstacle_cost_weight", params_.obstacle_cost_weight_, 1.0);
  // nh_.param("learning_rate", params_.learning_rate_, 0.01);
  // nh_.param("animate_path", params_.animate_path_, true);
  // nh_.param("add_randomness", params_.add_randomness_, false);
  // nh_.param("smoothness_cost_velocity", params_.smoothness_cost_velocity_, 0.0);
  // nh_.param("smoothness_cost_acceleration", params_.smoothness_cost_acceleration_, 1.0);
  // nh_.param("smoothness_cost_jerk", params_.smoothness_cost_jerk_, 0.0);
  // nh_.param("hmc_discretization", params_.hmc_discretization_, 0.01);
  // nh_.param("hmc_stochasticity", params_.hmc_stochasticity_, 0.01);
  // nh_.param("hmc_annealing_factor", params_.hmc_annealing_factor_, 0.99);
  // nh_.param("use_hamiltonian_monte_carlo", params_.use_hamiltonian_monte_carlo_, false);
  // nh_.param("ridge_factor", params_.ridge_factor_, 0.0);
  // nh_.param("use_pseudo_inverse", params_.use_pseudo_inverse_, false);
  // nh_.param("pseudo_inverse_ridge_factor", params_.pseudo_inverse_ridge_factor_, 1e-4);
  // nh_.param("animate_endeffector", params_.animate_endeffector_, false);
  // nh_.param("animate_endeffector_segment", params_.animate_endeffector_segment_, std::string("r_gripper_tool_frame"));
  // nh_.param("joint_update_limit", params_.joint_update_limit_, 0.1);
  // nh_.param("collision_clearence", params_.min_clearence_, 0.2);
  // nh_.param("collision_threshold", params_.collision_threshold_, 0.07);
  // nh_.param("random_jump_amount", params_.random_jump_amount_, 1.0);
  // nh_.param("use_stochastic_descent", params_.use_stochastic_descent_, true);
  //filter_mode_ = false;
}

bool SqppPlanner::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                         const moveit_msgs::GetMotionPlan::Request &req, 
                         const chomp::SqppParameters& params,
                         moveit_msgs::GetMotionPlan::Response &res) const
{
  ros::WallTime start_time = ros::WallTime::now()
    //Helper class for storing/generating an initial trajectory
    // (why does it need the kinematic model?)
  ChompTrajectory trajectory(planning_scene->getKinematicModel(),
                             3.0,
                             .03,
                             req.motion_plan_request.group_name);
  //Gets the initial joint state from the motion plan request, puts it in trajectory[0]
  jointStateToArray(planning_scene->getKinematicModel(),
                    req.motion_plan_request.start_state.joint_state, 
                    req.motion_plan_request.group_name,
                    trajectory.getTrajectoryPoint(0));

  int goal_index = trajectory.getNumPoints()- 1;
  // Sets the end state to the initial state
  trajectory.getTrajectoryPoint(goal_index) = trajectory.getTrajectoryPoint(0);
  sensor_msgs::JointState js;

  // Gathers the goal joint constraints into a JointState object
  for(unsigned int i = 0; i < req.motion_plan_request.goal_constraints[0].joint_constraints.size(); i++) {
    js.name.push_back(req.motion_plan_request.goal_constraints[0].joint_constraints[i].joint_name);
    js.position.push_back(req.motion_plan_request.goal_constraints[0].joint_constraints[i].position);
    ROS_INFO_STREAM("Setting joint " << req.motion_plan_request.goal_constraints[0].joint_constraints[i].joint_name
                    << " to position " << req.motion_plan_request.goal_constraints[0].joint_constraints[i].position);
  }

  // Sets the finalposition in the initial trajectory to the goal joint state
  jointStateToArray(planning_scene->getKinematicModel(),
                    js, 
                    req.motion_plan_request.group_name, 
                    trajectory.getTrajectoryPoint(goal_index));
  const planning_models::KinematicModel::JointModelGroup* model_group = 
    planning_scene->getKinematicModel()->getJointModelGroup(req.motion_plan_request.group_name);
  // fix the goal to move the shortest angular distance for wrap-around joints:
  for (size_t i = 0; i < model_group->getJointModels().size(); i++)
  {
    const planning_models::KinematicModel::JointModel* model = model_group->getJointModels()[i];
    const planning_models::KinematicModel::RevoluteJointModel* revolute_joint = dynamic_cast<const planning_models::KinematicModel::RevoluteJointModel*>(model);

    if (revolute_joint != NULL)
    {
      if(revolute_joint->isContinuous())
      {
        double start = (trajectory)(0, i);
        double end = (trajectory)(goal_index, i);
        ROS_INFO_STREAM("Start is " << start << " end " << end << " short " << shortestAngularDistance(start, end));
        (trajectory)(goal_index, i) = start + shortestAngularDistance(start, end);
      }
    }
  }
  
  // fill in an initial quintic spline trajectory
  trajectory.fillInMinJerk();

  // optimize!
  planning_models::KinematicState start_state(planning_scene->getCurrentState());
  planning_models::robotStateToKinematicState(*planning_scene->getTransforms(), req.motion_plan_request.start_state, start_state);
    
  ros::WallTime create_time = ros::WallTime::now();
  // SqppOptimizer optimizer(&trajectory, 
  //                          planning_scene, 
  //                          req.motion_plan_request.group_name,
  //                          &params,
  //                          start_state);
  // if(!optimizer.isInitialized()) {
  //   ROS_WARN_STREAM("Could not initialize optimizer");
  //   res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
  //   return false;
  // }
  // ROS_INFO("Optimization took %f sec to create", (ros::WallTime::now() - create_time).toSec());
  // ROS_INFO("Optimization took %f sec to create", (ros::WallTime::now() - create_time).toSec());
  // optimizer.optimize();
  // ROS_INFO("Optimization actually took %f sec to run", (ros::WallTime::now() - create_time).toSec());
  create_time = ros::WallTime::now();
  // assume that the trajectory is now optimized, fill in the output structure:

  ROS_INFO("Output trajectory has %d joints", trajectory.getNumJoints());

  // fill in joint names:
  res.trajectory.joint_trajectory.joint_names.resize(trajectory.getNumJoints());
  for (size_t i = 0; i < model_group->getJointModels().size(); i++)
  {
    res.trajectory.joint_trajectory.joint_names[i] = model_group->getJointModels()[i]->getName();
  }

  res.trajectory.joint_trajectory.header = req.motion_plan_request.start_state.joint_state.header; // @TODO this is probably a hack

  // fill in the entire trajectory
  res.trajectory.joint_trajectory.points.resize(trajectory.getNumPoints());
  for (int i=0; i < trajectory.getNumPoints(); i++)
  {
    res.trajectory.joint_trajectory.points[i].positions.resize(trajectory.getNumJoints());
    for (size_t j=0; j < res.trajectory.joint_trajectory.points[i].positions.size(); j++)
    {
      res.trajectory.joint_trajectory.points[i].positions[j] = trajectory.getTrajectoryPoint(i)(j);
      if(i == trajectory.getNumPoints()-1) {
        ROS_INFO_STREAM("Joint " << j << " " << res.trajectory.joint_trajectory.points[i].positions[j]);
      }
    }
    // Setting invalid timestamps.
    // Further filtering is required to set valid timestamps accounting for velocity and acceleration constraints.
    res.trajectory.joint_trajectory.points[i].time_from_start = ros::Duration(0.0);
  }
  
  ROS_INFO("Bottom took %f sec to create", (ros::WallTime::now() - create_time).toSec());
  ROS_INFO("Serviced planning request in %f wall-seconds, trajectory duration is %f", (ros::WallTime::now() - start_time).toSec(), res.trajectory.joint_trajectory.points[goal_index].time_from_start.toSec());
  res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  res.planning_time = ros::Duration((ros::WallTime::now() - start_time).toSec());
  return true;
}

}
