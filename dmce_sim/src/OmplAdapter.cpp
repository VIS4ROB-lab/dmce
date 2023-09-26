#include "dmce_sim/OmplAdapter.hpp"

#include <tf_conversions/tf_eigen.h>

namespace dmce
{

OmplAdapter::OmplAdapter()
    : costmap_(NULL), initialized_(false), has_odometry_(false)
{
}

OmplAdapter::OmplAdapter(std::string name, costmap_2d::Costmap2D* costmap,
                         std::string frame_id)
    : costmap_(NULL), initialized_(false), has_odometry_(false)
{
  // initialize the planner
  initialize(name, costmap, frame_id);
}

OmplAdapter::~OmplAdapter() {}

void OmplAdapter::initialize(std::string name, costmap_2d::Costmap2D* costmap,
                             std::string frame_id)
{
  if (!initialized_)
  {
    // Set up ROS and parameters
    ros::NodeHandle private_nh("~/" + name);
    costmap_ = costmap;
    frame_id_ = frame_id;

    private_nh.param("old_navfn_behavior", old_navfn_behavior_, false);
    if (!old_navfn_behavior_)
      convert_offset_ = 0.5;
    else
      convert_offset_ = 0.0;

    // Now set up the planner
    smb_ompl_planner::RrtParameters rrt_params;
    rrt_params.use_distance_threshold = true;
    rrt_params.simplify_solution = true;
    rrt_params.trust_approx_solution = false;
    rrt_params.verbose_planner = false;

    rrt_params.distance_threshold = 1.1;
    rrt_params.goal_bias = 0.05;
    rrt_params.tree_range = 1;
    rrt_params.interpolation_factor = 0.05;
    rrt_params.num_seconds_to_plan = 1.0;
    dist_goal_reached_ = 0.3;
    default_tolerance_ = 0.0;
    min_distance_waypoints_ = 0.1;

    int planner_type;
    planner_type = 1;

    bool enable_timer_collisions;
    enable_timer_collisions = false;

    double replanning_rate;
    replanning_rate = 5.0;

    // std::string odometry_topic;
    // if (!private_nh.getParam("odometry_topic", odometry_topic))
    // {
    //   ROS_WARN("[Ompl Planner] Odometry topic not specified");
    //   odometry_topic = "/odometry_test";
    // }
    // else
    // {
    //   ROS_INFO_STREAM(
    //       "[Ompl Planner] Subscribing to odom topic: " << odometry_topic);
    // }

    // ROS communication
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
    // odometry_sub_ = private_nh.subscribe(odometry_topic, 10,
    //                                      &OmplAdapter::odometryCallback, this);
    // make_plan_srv_ = private_nh.advertiseService(
    //     "make_plan", &OmplAdapter::makePlanService, this);

    // Now start the ROS timer - every time the path is in collision, we replan
    // if (enable_timer_collisions)
    // {
    //   timer_collisions_ =
    //       private_nh.createTimer(ros::Duration(1 / replanning_rate),
    //                              &OmplAdapter::collisionTimerCallback, this);
    // }

    ompl_planner_ = std::make_shared<smb_ompl_planner::RrtPlanner>(
        rrt_params, smb_ompl_planner::RrtPlannerType(planner_type));
    orientation_filter_ = std::make_shared<smb_ompl_planner::OrientationFilter>();

    // Done
    initialized_ = true;
  }
  else
  {
    ROS_WARN("[Ompl Planner] This planner has already been initialized, you "
             "can't call it twice, doing nothing");
  }
}

void OmplAdapter::clearRobotCell(const geometry_msgs::PoseStamped& global_pose,
                                 unsigned int mx, unsigned int my)
{
  if (!initialized_)
  {
    ROS_ERROR("[Ompl Planner] his planner has not been initialized yet, but it "
              "is being used, please call initialize() before use");
    return;
  }

  // set the associated costs in the cost map to be free
  costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}

void OmplAdapter::mapToWorld(double mx, double my, double& wx, double& wy)
{
  wx = costmap_->getOriginX() +
       (mx + convert_offset_) * costmap_->getResolution();
  wy = costmap_->getOriginY() +
       (my + convert_offset_) * costmap_->getResolution();
}

bool OmplAdapter::worldToMap(double wx, double wy, double& mx, double& my)
{
  double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
  double resolution = costmap_->getResolution();

  if (wx < origin_x || wy < origin_y)
    return false;

  mx = (wx - origin_x) / resolution - convert_offset_;
  my = (wy - origin_y) / resolution - convert_offset_;

  if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
    return true;

  return false;
}

bool OmplAdapter::makePlan(const geometry_msgs::PoseStamped& start,
                           const geometry_msgs::PoseStamped& goal,
                           std::vector<geometry_msgs::PoseStamped>& plan)
{
  return makePlan(start, goal, default_tolerance_, plan);
}

bool OmplAdapter::makePlan(const geometry_msgs::PoseStamped& start,
                           const geometry_msgs::PoseStamped& goal,
                           const double tolerance,
                           std::vector<geometry_msgs::PoseStamped>& plan)
{
  boost::mutex::scoped_lock lock(mutex_);
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized yet, but it is being "
              "used, please call initialize() before use");
    return false;
  }

  // if (!has_odometry_)
  // {
  //   ROS_ERROR_THROTTLE(5, "[Ompl Planner] Odometry not available. Have you set "
  //                         "the right topic in the parameter file?");
  //   return false;
  // }

  // clear the plan, just in case
  plan.clear();

  // until tf can handle transforming things that are way in the past... we'll
  // require the goal to be in our global frame
  std::string global_frame = frame_id_;
  if (goal.header.frame_id != global_frame)
  {
    ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  "
              "It is instead in the %s frame.",
              global_frame.c_str(), goal.header.frame_id.c_str());
    return false;
  }

  if (start.header.frame_id != global_frame)
  {
    ROS_ERROR("The start pose passed to this planner must be in the %s frame.  "
              "It is instead in the %s frame.",
              global_frame.c_str(), start.header.frame_id.c_str());
    return false;
  }

  double wx = start.pose.position.x;
  double wy = start.pose.position.y;

  unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
  double start_x, start_y, goal_x, goal_y;

  if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i))
  {
    ROS_WARN(
        "The robot's start position is off the global costmap. Planning will "
        "always fail, are you sure the robot has been properly localized?");
    // ROS_WARN("wx: %f \n wy: %f \n mx: %d \n my: %d", wx, wy, start_x_i, start_y_i);
    // int x, y;
    // costmap_->worldToMapNoBounds(wx, wy, x, y);
    // ROS_WARN("ox: %f \n oy: %f \n x: %d \n y: %d", costmap_->getOriginX(), costmap_->getOriginY(), x, y);
    // ROS_WARN("sx: %d \n sy: %d \n smx: %f \n smy: %f", costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY(), costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
    return false;
  }
  if (old_navfn_behavior_)
  {
    start_x = start_x_i;
    start_y = start_y_i;
  }
  else
  {
    worldToMap(wx, wy, start_x, start_y);
  }

  wx = goal.pose.position.x;
  wy = goal.pose.position.y;

  if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i))
  {
    ROS_WARN_THROTTLE(1.0,
                      "The goal sent to the global planner is off the global "
                      "costmap. Planning will always fail to this goal.");
    return false;
  }
  if (old_navfn_behavior_)
  {
    goal_x = goal_x_i;
    goal_y = goal_y_i;
  }
  else
  {
    worldToMap(wx, wy, goal_x, goal_y);
  }

  // clear the starting cell within the costmap because we know it can't be an
  // obstacle
  clearRobotCell(start, start_x_i, start_y_i);

  int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
  double costmap_size_x = costmap_->getSizeInMetersX();
  double costmap_size_y = costmap_->getSizeInMetersY();
  double cell_size =
      std::max(costmap_size_x / double(nx), costmap_size_y / double(ny));

  Eigen::Vector2d lower_bound, upper_bound;
  lower_bound << -costmap_size_x / 2.0, -costmap_size_y / 2.0;
  upper_bound << costmap_size_x / 2.0, costmap_size_y / 2.0;

  // Set up the problem first
  Eigen::Vector2d start_eigen(start.pose.position.x, start.pose.position.y);
  Eigen::Vector2d goal_eigen(goal.pose.position.x, goal.pose.position.y);
  ompl_planner_->setBounds(lower_bound, upper_bound);
  ompl_planner_->setVoxelSize(cell_size);
  ompl_planner_->setCostmap(costmap_);
  ompl_planner_->setupProblem(start_eigen, goal_eigen);

  // Planning here!
  std::vector<Eigen::Vector2d> ompl_path;
  bool success;
  try
  {
    success = ompl_planner_->getPathBetweenWaypoints(start_eigen, goal_eigen,
                                                     ompl_path);
  }
  catch (ompl::Exception& ex)
  {
    ROS_ERROR("[Ompl Planner] Ompl exception: %s", ex.what());
    return false;
  }

  if (!success)
  {
    ROS_ERROR("[Ompl Planner] Planning failed");
    return false;
  }

  ROS_INFO("[Ompl Planner] Planning successful");

  // Interpolate the global path in between
  global_path_.clear();
  global_path_.push_back(ompl_path[0]);

  std::vector<double> orientations;
  orientations.push_back(tf::getYaw(start.pose.orientation));

  for (size_t i = 1; i < ompl_path.size(); ++i)
  {
    Eigen::Vector2d direction(ompl_path[i] - ompl_path[i - 1]);
    double distance_path_chunk(direction.norm());
    int n_points = std::ceil(distance_path_chunk / min_distance_waypoints_);

    for (int j = 1; j <= n_points; ++j) // skip the first one (avoid repetition)
    {
      Eigen::Vector2d wp =
          ompl_path[i - 1] + double(j) / double(n_points) * direction;
      global_path_.push_back(wp);

      Eigen::Vector2d direction_normalized(direction.normalized());
      orientations.push_back(
          std::atan2(direction_normalized.y(), direction_normalized.x()));
    }
  }

  // Save the path in the right format
  geometry_msgs::PoseStamped pose_path = start;
  pose_path.header.stamp = ros::Time::now();
  plan.push_back(pose_path);
  for (size_t i = 0; i < global_path_.size(); ++i)
  {
    Eigen::Vector2d waypoint = global_path_[i];
    pose_path.pose.position.x = waypoint.x();
    pose_path.pose.position.y = waypoint.y();

    // Set the right orientation for every point (the last point has to have
    // the goal orientation!)
    if (i != global_path_.size() - 1)
    {
      pose_path.pose.orientation =
          tf::createQuaternionMsgFromYaw(orientations[i]);
    }
    else
    {
      pose_path.pose.orientation = goal.pose.orientation;
    }

    pose_path.header.stamp = ros::Time::now();
    plan.push_back(pose_path);
  }

  // add orientations if needed
  orientation_filter_->processPath(start, plan);

  // publish the plan for visualization purposes
  // publishPlan(plan);

  // Update variables for the checks
  goal_ = goal_eigen;

  return true;
}

// void OmplAdapter::publishPlan(
//     const std::vector<geometry_msgs::PoseStamped>& path)
// {
//   if (!initialized_)
//   {
//     ROS_ERROR("[Ompl Planner] This planner has not been initialized yet, but "
//               "it is being used, please call initialize() before use");
//     return;
//   }

//   // create a message for the plan
//   nav_msgs::Path gui_path;
//   gui_path.poses.resize(path.size());

//   gui_path.header.frame_id = frame_id_;
//   gui_path.header.stamp = ros::Time::now();

//   // Extract the plan in world co-ordinates, we assume the path is all in the
//   // same frame
//   for (unsigned int i = 0; i < path.size(); i++)
//   {
//     gui_path.poses[i] = path[i];
//   }

//   plan_pub_.publish(gui_path);
// }

void OmplAdapter::outlineMap(unsigned char* costarr, int nx, int ny,
                             unsigned char value)
{
  unsigned char* pc = costarr;
  for (int i = 0; i < nx; i++)
    *pc++ = value;
  pc = costarr + (ny - 1) * nx;
  for (int i = 0; i < nx; i++)
    *pc++ = value;
  pc = costarr;
  for (int i = 0; i < ny; i++, pc += nx)
    *pc = value;
  pc = costarr + nx - 1;
  for (int i = 0; i < ny; i++, pc += nx)
    *pc = value;
}

} // end namespace smb_ompl_planner

