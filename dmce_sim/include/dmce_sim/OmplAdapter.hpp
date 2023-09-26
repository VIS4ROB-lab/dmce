#pragma once

#include <vector>

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include "smb_ompl_planner/orientation_filter.h"
#include "smb_ompl_planner/rrt_planner.h"

namespace dmce
{

class OmplAdapter
{
public:
  /**
   * @brief  Default constructor for the OmplAdapter object
   */
  OmplAdapter();

  /**
   * @brief  Constructor for the PlannerCore object
   * @param  name The name of this planner
   * @param  costmap A pointer to the costmap to use
   * @param  frame_id Frame of the costmap
   */
  OmplAdapter(std::string name, costmap_2d::Costmap2D* costmap,
              std::string frame_id);

  /**
   * @brief  Default deconstructor for the PlannerCore object
   */
  ~OmplAdapter();

  /**
   * @brief  Initialization function for the PlannerCore object
   * @param  name The name of this planner
   * @param  costmap A pointer to the costmap to use for planning
   * @param frame_id TF frame ID of the map
   */
  void initialize(std::string name, costmap_2d::Costmap2D* costmap,
                  std::string frame_id);

  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose
   * @param goal The goal pose
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);

  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose
   * @param goal The goal pose
   * @param tolerance The tolerance on the goal point for the planner
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal, const double tolerance,
                std::vector<geometry_msgs::PoseStamped>& plan);
  /**
   * @brief  Publish a path for visualization purposes
   */
  // void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

  /**
   * @brief Callbacks
   */
  // bool makePlanService(nav_msgs::GetPlan::Request& req,
  //                      nav_msgs::GetPlan::Response& resp);
  // void odometryCallback(const nav_msgs::OdometryConstPtr& odom_msg);
  // void collisionTimerCallback(const ros::TimerEvent&);

protected:
  /**
   * @brief Store a copy of the current costmap in \a costmap.  Called by
   * makePlan.
   */
  costmap_2d::Costmap2D* costmap_;
  std::string frame_id_;

  ros::Publisher plan_pub_;
  ros::Subscriber odometry_sub_;
  ros::Timer timer_collisions_;

  bool initialized_;
  bool has_odometry_;

  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>
      global_path_;
  Eigen::Vector2d odometry_;
  Eigen::Vector2d goal_;

private:
  void mapToWorld(double mx, double my, double& wx, double& wy);
  bool worldToMap(double wx, double wy, double& mx, double& my);
  void clearRobotCell(const geometry_msgs::PoseStamped& global_pose,
                      unsigned int mx, unsigned int my);
  void outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value);

  // Planner
  std::shared_ptr<smb_ompl_planner::RrtPlanner> ompl_planner_;
  std::shared_ptr<smb_ompl_planner::OrientationFilter> orientation_filter_;

  // Utilities
  boost::mutex mutex_;
  ros::ServiceServer make_plan_srv_;

  unsigned char* cost_array_;
  unsigned int start_x_, start_y_, end_x_, end_y_;

  double default_tolerance_;
  double min_distance_waypoints_;
  double dist_goal_reached_;

  bool old_navfn_behavior_;
  float convert_offset_;
}; // end class OmplAdapter

} // end namespace dmce
