#include "hfn.hpp"

#include <tf/tf.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <kr_traj_msgs/PolyTrajByCoeffs.h>

#include <boost/thread/thread.hpp>

#include <CGAL/squared_distance_2.h>

#include <angles/angles.h>

using namespace std;

//=========================== Helper functions ============================//

double linear_distance(const geometry_msgs::Pose &start,
                       const geometry_msgs::Pose &stop) {
  return hypot(start.position.x - stop.position.x,
               start.position.y - stop.position.y);
}

double ang_distance(const geometry_msgs::Pose &start,
                    const geometry_msgs::Pose &stop) {
  return fabs(angles::shortest_angular_distance(tf::getYaw(start.orientation),
                                                tf::getYaw(stop.orientation)));
}

//=========================== HumanFriendlyNav ============================//
namespace scarab {

HumanFriendlyNav::HumanFriendlyNav(Params p) : params_(p) {

}

HumanFriendlyNav* HumanFriendlyNav::ROSInit(ros::NodeHandle& nh) {
  Params p;
  nh.param("axle_width", p.axle_width, 0.255);
  nh.param("robot_radius", p.robot_radius, 0.23);
  nh.param("safety_margin", p.safety_margin, 0.10);
  nh.param("social_margin", p.social_margin, 0.2);

  nh.param("freq", p.freq, 5.0);
  nh.param("base_frame_id", p.base_frame, string("base"));
  nh.param("map_frame_id", p.map_frame, string("/map"));

  nh.param("tau_1", p.tau_1, 2.0);
  nh.param("tau_2", p.tau_2, 0.25);
  nh.param("tau_r", p.tau_r, 1.0);

  nh.param("v_opt", p.v_opt, 0.5);
  nh.param("w_max", p.w_max, 0.7);
  nh.param("waypoint_thresh", p.waypoint_thresh, 0.2);
  nh.param("alpha_thresh", p.alpha_thresh, 2.094);

  HumanFriendlyNav *human_friendly_nav = new HumanFriendlyNav(p);
  return human_friendly_nav;
}

void HumanFriendlyNav::setLaserScan(const sensor_msgs::LaserScan &input) {
  double alpha_des, distance_des;
  freeDistance(input);
  desiredOrientation(alpha_des, distance_des);
  orientationToTwist(alpha_des, distance_des, goal_twist_);
}

void HumanFriendlyNav::setPose(const geometry_msgs::PoseStamped &input) {
  if (input.header.frame_id != params_.map_frame) {
    ROS_ERROR("HumanFriendlyNav::setPose() Must have pose in map frame.  Map frame = %s, pose frame = %s ",
              params_.map_frame.c_str(), input.header.frame_id.c_str());
    ROS_BREAK();
  }
  pose_ = input.pose;
}

void HumanFriendlyNav::setGoal(const geometry_msgs::PoseStamped &input) {
  // Update goal_, make sure it's in local frame
  if (input.header.frame_id == params_.map_frame) {
    tf::Transform global_to_local_tf;
    tf::poseMsgToTF(pose_, global_to_local_tf);

    tf::Transform goal_global;
    tf::poseMsgToTF(input.pose, goal_global);

    tf::poseTFToMsg(global_to_local_tf.inverse() * goal_global, goal_);
  } else if (input.header.frame_id == params_.base_frame) {
    goal_ = input.pose;
  } else {
    ROS_ERROR("Goal pose is in frame: %s but expected global frame: %s or local frame: %s",
              input.header.frame_id.c_str(), params_.map_frame.c_str(),
              params_.base_frame.c_str());
    ROS_BREAK();
  }
  goal_.orientation.x = 0.0;
  goal_.orientation.y = 0.0;
  goal_.orientation.z = 0.0;
  goal_.orientation.w = 1.0;

  // Move goal_ to closest point on interior of polygon defined if it's on the
  // outside
  Point_2 old_goal(goal_.position.x, goal_.position.y);
  if (CGAL::bounded_side_2(polygon_.vertices_begin(), polygon_.vertices_end(),
                           old_goal, K()) == CGAL::ON_UNBOUNDED_SIDE) {
    // Find closest vertex
    Polygon_2::Vertex_iterator closest_v, it_v;
    double min_dist_vertex = numeric_limits<double>::max();
    for (it_v = polygon_.vertices_begin(); it_v != polygon_.vertices_end(); ++it_v) {
      double dist = CGAL::squared_distance(old_goal, *it_v);
      if (dist < min_dist_vertex) {
        closest_v = it_v;
        min_dist_vertex = dist;
      }
    }

    // Find closest edge
    Polygon_2::Edge_const_iterator closest_e, it_e;
    double min_dist_edge = numeric_limits<double>::max();
    for (it_e = polygon_.edges_begin(); it_e != polygon_.edges_end(); ++it_e) {
      double dist = CGAL::squared_distance(old_goal, *it_e);
      if (dist < min_dist_edge) {
        closest_e = it_e;
        min_dist_edge = dist;
      }
    }

    // Pick object that's closet
    if (min_dist_vertex < min_dist_edge) {
      goal_.position.x = closest_v->x();
      goal_.position.y = closest_v->y();
    } else {
      // Check if point when projected onto segment lies on segment
      Vector_2 seg_vec = closest_e->to_vector();
      Vector_2 point_vec(closest_e->source(), old_goal);

      double dotprod = seg_vec * point_vec;
      if (0.0 <= dotprod && dotprod <= seg_vec.squared_length()) {
        // Point lies on segment
        Vector_2 projection = dotprod / seg_vec.squared_length() * seg_vec;
        goal_.position.x = closest_e->source().x() + projection.x();
        goal_.position.y = closest_e->source().y() + projection.y();
      } else {
        // Point is not on segment, use closest vertex instead
        goal_.position.x = closest_v->x();
        goal_.position.y = closest_v->y();
      }
    }

    Point_2 boundary_point(goal_.position.x, goal_.position.y);
    Vector_2 direction(old_goal, boundary_point);
    Vector_2 offset = direction / sqrt(direction.squared_length());
    offset = offset * 0.8 * params_.waypoint_thresh;
    goal_.position.x += offset.x();
    goal_.position.y += offset.y();
  }

  Point_2 new_goal = Point_2(goal_.position.x, goal_.position.y);
  if (CGAL::bounded_side_2(polygon_.vertices_begin(), polygon_.vertices_end(),
                           new_goal, K()) == CGAL::ON_UNBOUNDED_SIDE) {
    // ROS_WARN("Failed to move point to inside");
    // ROS_BREAK();
  }
}

void HumanFriendlyNav::setOdom(const nav_msgs::Odometry &input) {
  current_twist_ = input.twist.twist;
}

void HumanFriendlyNav::freeDistance(const sensor_msgs::LaserScan &input) {
  free_distance_ = input;

  vector<float>::const_iterator it_input;
  vector<float>::iterator it_free;
  for (it_input=input.ranges.begin(), it_free=free_distance_.ranges.begin();
       it_input!=input.ranges.end(); ++it_input, ++it_free) {
    if (input.range_min <= *it_input && *it_input <= input.range_max) {
      addObstacle(it_free, *it_input);
    }
  }

  // Build polygon
  polygon_.clear();
  polygon_.push_back(Point_2(0.0, 0.0));
  for (size_t i = 0; i < free_distance_.ranges.size(); ++i) {
    double orig_range = input.ranges.at(i);
    if (input.range_min <= orig_range && orig_range <= input.range_max) {
      double t = free_distance_.angle_min + i * free_distance_.angle_increment;
      double x = cos(t) * free_distance_.ranges.at(i);
      double y = sin(t) * free_distance_.ranges.at(i);
      polygon_.push_back(Point_2(x, y));
    }
  }
}

void HumanFriendlyNav::addObstacle(vector<float>::iterator &it, float scan_dist)
{
  float r = min(obstacleRadius(), scan_dist-0.001f);
  float phi = asin(r / scan_dist);
  float dTheta = free_distance_.angle_increment;
  float theta = 0.0;

  vector<float>::iterator it_f=it;
  while (!(it_f==free_distance_.ranges.begin() || theta-dTheta<=-phi)) {
    it_f--;
    theta -= dTheta;
  }

  while (it_f!=free_distance_.ranges.end() && theta+dTheta<=phi) {
    float l = calcReducedRange(scan_dist, r, theta);
    if (l < *it_f) {
      *it_f = l;
    }
    if (*it_f < 0.0) {
      *it_f = 0.0;
    }
    it_f++;
    theta += dTheta;
  }
}

float HumanFriendlyNav::calcReducedRange(float d, float r, float theta) {
  float b = -2.0*d*cos(theta);
  float c = d*d - r*r;

  float discriminant = b*b - 4.0*c;
  if (discriminant < 0.0) {
    return 0.0;
  }

  float l = (-b-sqrt(discriminant)) / 2.0;
  return (l>0.0) ? l : 0.0;
}

float HumanFriendlyNav::obstacleRadius() {
  return params_.robot_radius + params_.safety_margin;
}

void HumanFriendlyNav::desiredOrientation(double &alpha_des, double &distance_des) {
  vector<float>::const_iterator it;
  double alpha;

  // get distance directly in front of robot
  size_t ind = free_distance_.ranges.size()/2;
  double distance0 = free_distance_.ranges[ind];

  // if goal within sight, drive towards it
  double alpha_goal = atan2(goal_.position.y, goal_.position.x);
  if ((alpha_goal>free_distance_.angle_min) &&
      (alpha_goal<free_distance_.angle_max)) {
    it = free_distance_.ranges.begin();
    alpha = free_distance_.angle_min;
    while (alpha_goal > alpha + free_distance_.angle_increment) {
      alpha += free_distance_.angle_increment;
      it++;
    }

    geometry_msgs::Pose zero_pose;
    double distance_to_goal = linear_distance(goal_, zero_pose);
    if (distance_to_goal <= *it) {
      alpha_des = alpha_goal;
      distance_des = min(distance_to_goal, distance0);
      return;
    }
  }

  // else drive towards closest free point
  alpha_des = 0;
  double distance_to_goal = numeric_limits<double>::max();

  alpha = free_distance_.angle_min;
  for (it=free_distance_.ranges.begin(); it!=free_distance_.ranges.end();
       ++it, alpha+=free_distance_.angle_increment) {
    geometry_msgs::Pose scan_point;
    scan_point.position.x = (*it) * cos(alpha);
    scan_point.position.y = (*it) * sin(alpha);

    double dist = linear_distance(goal_, scan_point);
    if (dist < distance_to_goal) {
      distance_to_goal = dist;
      alpha_des = alpha;
      distance_des = min(double(*it), distance0);
    }
  }
}

void HumanFriendlyNav::orientationToTwist(double alpha, double distance,
                                          geometry_msgs::Twist &twist) {
  twist.linear.x = desiredVelocity(distance, alpha);

  double w = alpha/params_.tau_r;
  if (fabs(w) > params_.w_max) {
    twist.angular.z = copysign(params_.w_max, w);
  } else {
    twist.angular.z = w;
  }
}

double HumanFriendlyNav::desiredVelocity(double distance, double alpha) {
  double v_des = min(params_.v_opt, distance / params_.tau_1);
  if (fabs(alpha) > params_.alpha_thresh) {
    return 0.0;
  } else {
    return v_des * (1.0 - fabs(alpha) / params_.alpha_thresh);
  }
}

double clamp(double val, double min, double max) {
  if (val < min) {
    return min;
  } else if (val > max) {
    return max;
  } else {
    return val;
  }
}

void HumanFriendlyNav::getCommandVel(geometry_msgs::Twist *cmd_vel)
{
  double left, right;

  //std::cout << "the current twist is " << current_twist_.linear.x << " " << current_twist_.angular.z << std::endl;
  twistToWheelVel(current_twist_, left, right);

  double left_g, right_g;
  twistToWheelVel(goal_twist_, left_g, right_g);

  double left_dot = (left_g - left) / params_.tau_2;
  double right_dot = (right_g - right) / params_.tau_2;

  // change this to be limited by max accelleration
  left += left_dot / params_.freq;
  right += right_dot / params_.freq;

  // Wheel vel to twist
  cmd_vel->linear.x = (right + left) / 2.0;
  //cmd_vel->angular.z = (right -left) / params_.axle_width;
  cmd_vel->angular.z = 0.0;
}


void HumanFriendlyNav::twistToWheelVel(const geometry_msgs::Twist &twist,
                                       double &left, double &right) {
  left = twist.linear.x - twist.angular.z*params_.axle_width/2.0;
  right = twist.linear.x + twist.angular.z*params_.axle_width/2.0;
}

//=============================== HFNWrapper ================================//


HFNWrapper::HFNWrapper(const Params &params, HumanFriendlyNav *hfn) :
  active_(false), turning_(false), map_(new scarab::OccupancyMap()),
  params_(params), hfn_(hfn) {
  flags_.have_pose = false;
  flags_.have_odom = false;
  flags_.have_map = false;
  flags_.have_laser = false;

  path_pub_ = nh_.advertise<nav_msgs::Path>("path", 5, true);
  vis_pub_ = nh_.advertise<visualization_msgs::Marker>("marker", 10, true);
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  inflated_pub_ = nh_.advertise<sensor_msgs::LaserScan>("inflated_scan", 10, true);
  costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("costmap", 1, true);
  traj_pub_ = nh_.advertise<visualization_msgs::Marker>("traj", 10, true);
  poly_pub_ = nh_.advertise<kr_traj_msgs::PolyTrajByCoeffs>("poly_traj_by_coeffs", 10, true);


  pose_sub_ = nh_.subscribe("pose", 1, &HFNWrapper::onPose, this);
  map_sub_ = nh_.subscribe("map", 1, &HFNWrapper::onMap, this);
  laser_sub_ = nh_.subscribe("scan", 1, &HFNWrapper::onLaserScan, this);
  odom_sub_ = nh_.subscribe("odom", 1, &HFNWrapper::onOdom, this);

  map_->setThresholds(params_.free_threshold, params_.occupied_threshold);
  map_->setMapFrameID(params_.map_frame);
  

  traj_gen_.reset(new TrajectoryGenerator(1, 2));

  pubWaypoints();
}

HFNWrapper::~HFNWrapper() {
}


void HFNWrapper::registerStatusCallback(const boost::function<void(Status)> &callback) {
  callback_ = callback;
}

HFNWrapper* HFNWrapper::ROSInit(ros::NodeHandle& nh) {
  Params p;
  nh.param("max_occ_dist", p.max_occ_dist, 0.5);
  nh.param("lethal_occ_dist", p.lethal_occ_dist, 0.23);
  nh.param("cost_occ_prob", p.cost_occ_prob, 0.0);
  nh.param("cost_occ_dist", p.cost_occ_dist, 0.0);
  nh.param("goal_tolerance", p.goal_tol, 0.2);
  nh.param("goal_tolerance_ang", p.goal_tol_ang, M_PI * 2.0);
  nh.param("path_margin", p.path_margin, 0.5);
  nh.param("waypoint_spacing", p.waypoint_spacing, 0.05);
  nh.param("los_margin", p.los_margin, 0.2);
  nh.param("timeout", p.timeout, 500.0);
  nh.param("stuck_distance", p.stuck_distance, 0.05);
  nh.param("stuck_angle", p.stuck_angle, 0.05);
  nh.param("stuck_timeout", p.stuck_timeout, 5.0);
  nh.param("stuck_start", p.stuck_start, 1.0);
  nh.param("free_threshold", p.free_threshold, 0);
  nh.param("occupied_threshold", p.occupied_threshold, 100);
  nh.param("allow_unknown_path", p.allow_unknown_path, true);
  nh.param("allow_unknown_los", p.allow_unknown_los, false);
  nh.param("map_frame_id", p.map_frame, string("/map"));
  nh.param("min_map_update", p.min_map_update, 0.0);
  nh.param("traj_mode", p.traj_mode, 0);
  nh.param("max_speed", p.max_speed, 0.5);
  nh.param("max_acc", p.max_acc, 0.5);
  nh.param("agent", p.agent, string("scarab"));
  nh.param("d_yaw", p.d_yaw, 0.0);
  nh.param("agent_r", p.agent_r, 0.0);
  nh.param("robot_id", p.robot_id, 0);


  p.name_space = nh.getNamespace();

  HumanFriendlyNav *hfn = HumanFriendlyNav::ROSInit(nh);

  HFNWrapper *wrapper = new HFNWrapper(p, hfn);
  return wrapper;
}

void HFNWrapper::ensureValidPose() {
  if (flags_.have_map && flags_.have_pose) {
    double startx = pose_.pose.position.x;
    double starty = pose_.pose.position.y;
    double newx, newy;
    bool valid =
      map_->nearestPoint(startx, starty, params_.lethal_occ_dist,
                         &newx, &newy);
    if (!valid) {
      ROS_WARN("Couldn't find valid starting position from (% .2f, % .2f)",
               startx, starty);
    } else if (startx != newx || starty != newy) {
      ROS_WARN_THROTTLE(5.0, "Shifted position from (% .2f, % .2f) to (% .2f, % .2f)",
               startx, starty, newx, newy);
      pose_.pose.position.x = newx;
      pose_.pose.position.y = newy;
      hfn_->setPose(pose_);
    }
  }
}


void HFNWrapper::onPose(const geometry_msgs::PoseStamped &input) {
  pose_ = input;
  flags_.have_pose = true;
  hfn_->setPose(pose_);

  ensureValidPose();

  if (!initialized()) {
    return;
  }

  pose_history_.push_back(pose_);
  while ((pose_.header.stamp - pose_history_.front().header.stamp).toSec() > params_.stuck_timeout) {
    pose_history_.pop_front();
  }

  if (!active_) {
    return;
  }

  geometry_msgs::Twist cmd;
  // check if reached goal or stuck
  if (params_.traj_mode == 0)
  {

    hfn_->getCommandVel(&cmd);

  }else if (params_.traj_mode == 1)
  {
    hfn_->getCommandVel(&cmd);
    Eigen::Vector2f vel;
    double cur_time = (ros::Time::now() - traj_start_time_).toSec();
    cur_traj_.getVelocity( cur_time, vel);
    //cout << "traj vel = " << vel(0) << endl;
    cmd.linear.x = std::sqrt(vel(0) *  vel(0)  + vel(1) *  vel(1));
    if(ifMovingBack){
        cmd.linear.x = -cmd.linear.x;
    }
    cmd.angular.z = (init_yaw - cur_yaw > 0) ? params_.d_yaw : -params_.d_yaw;
  }

  bool xy_ok = turning_ ||
    linear_distance(pose_.pose, goals_.back().pose) < params_.goal_tol;
  if (xy_ok &&
      (params_.goal_tol_ang >= M_PI ||
      ang_distance(pose_.pose, goals_.back().pose) < params_.goal_tol_ang)) {
    stop();
    ROS_INFO("HFNWrapper: FINISHED");
    cur_traj_.clear();
    //callback_(FINISHED);
    move_back();
    //return;
  } else if ((ros::Time::now() - goal_time_).toSec() > params_.stuck_start) {
    //bool stuck = true;
      bool stuck = false;
    // Check if we've moved
    for (list<geometry_msgs::PoseStamped>::iterator it = pose_history_.begin();
        it != pose_history_.end(); ++it) {
      const geometry_msgs::PoseStamped &pose = pose_history_.front();
      if (linear_distance(pose.pose, it->pose) > params_.stuck_distance ||
          ang_distance(pose.pose, it->pose) > params_.stuck_angle) {
        stuck = false;
        break;
      }
    }
    if (stuck) {
      ROS_WARN("HFNWrapper: STUCK (Robot isn't moving)");
      stop();
      callback_(STUCK);
    }
  }
  
}

void HFNWrapper::onMap(const nav_msgs::OccupancyGrid &input) {
  geometry_msgs::PoseStamped goal;
  if ((input.header.stamp - last_map_update_).toSec() < params_.min_map_update) {
    ROS_DEBUG("HFNWrapper: NOT updating map!");
    return;
  }
  ROS_DEBUG("HFNWrapper: Updating map");
  last_map_update_ = ros::Time::now();
  flags_.have_map = true;
  map_->setMap(input);
  map_->updateCSpace(params_.max_occ_dist, params_.lethal_occ_dist,
                     params_.cost_occ_prob, params_.cost_occ_dist);
  if (costmap_pub_.getNumSubscribers() > 0) {
    costmap_pub_.publish(map_->getCostMap());
  }

  ensureValidPose();

  if (active_) {
    ROS_INFO("PLANNING1");
    //setGoal(goals_);
  }
}

void HFNWrapper::pubPolygon(const HumanFriendlyNav::Polygon_2 &polygon) {
  visualization_msgs::Marker m;
  m.header.stamp = ros::Time();
  m.header.frame_id = hfn_->params().base_frame;
  m.action = visualization_msgs::Marker::ADD;
  m.type = visualization_msgs::Marker::LINE_STRIP;
  m.id = 100;
  m.ns = params_.name_space + "/polygon";
  m.pose.orientation.w = 1.0;
  m.scale.x = 0.1;
  m.scale.y = 0.1;
  m.scale.z = 0.1;
  m.color.a = 1.0;
  m.color.r = 1.0;
  m.points.resize(polygon.size());
  HumanFriendlyNav::Polygon_2::Vertex_iterator it;
  size_t i;
  for (it = polygon.vertices_begin(), i = 0;
       it != polygon.vertices_end(); ++it, ++i) {
    m.points.at(i).x = it->x();
    m.points.at(i).y = it->y();
  }
  m.points.push_back(m.points.front());
  vis_pub_.publish(m);
}

string HFNWrapper::uninitializedString() {
  string description;
  if (!flags_.have_pose) { description += "pose "; }
  if (!flags_.have_odom) { description += "odom "; }
  if (!flags_.have_map) { description += "map "; }
  if (!flags_.have_laser) { description += "laser "; }
  return description;
}

void HFNWrapper::onLaserScan(const sensor_msgs::LaserScan &scan) {
  flags_.have_laser = true;
  hfn_->setLaserScan(scan);
  inflated_pub_.publish(hfn_->inflatedScan());

  pubPolygon(hfn_->inflatedPolygon());

  if (!initialized() || !active_) {
    return;
  }
  
  geometry_msgs::Twist cmd;
  if (params_.traj_mode == 1) {
    hfn_->getCommandVel(&cmd);
    Eigen::Vector2f vel, pos;
    if(ifMovingBack){
        cmd.linear.x = -cmd.linear.x;
    }
    cmd.angular.z = (init_yaw - cur_yaw > 0) ? params_.d_yaw : -params_.d_yaw;

    //params_.tau_2

    //to check if it's making turns
    double cur_time = (ros::Time::now() - traj_start_time_).toSec();
    //std::cout << "the current time is " << cur_time << std::endl;
    if (cur_time > cur_traj_.getTotalTime())
    {
      //cout << "move back at here\n";
      stop();
      cur_traj_.clear();
      //callback_(FINISHED);
      //return;
      move_back();
      return;
    }
    

    // odom and current position
    cur_traj_.getPosition(cur_time, pos);
    // std::cout << "the pos is " << pos << std::endl;
    // std::cout << "the cur_pos is " << cur_pos_ << std::endl;
    // std::cout << "the distance is " << (cur_pos_ -  pos).norm() << std::endl;
    // check velocity

    cur_traj_.getVelocity(cur_time, vel);
    //cur_traj_.getPosition(cur_time + 4, pos);
    //std::cout << "the pos is " << pos << std::endl;
    // std::cout << "vel is " << vel(0) << " " << vel(1) << std::endl;
    // std::cout << "the velocity is " << cmd.linear.x << std::endl;
    // std::cout << "!!!!!!!!!!!!!!!pos is " << pos(0) << " " << pos(1) << std::endl;
    // std::cout << " the angular velocity is " << cmd.angular.z << std::endl;
  
    //if vel is nan, then stop
    if (std::isnan(vel(0)) || std::isnan(vel(1)))
    {
      stop();
      cur_traj_.clear();
      //callback_(FINISHED);
      //return;
      move_back();
      return;
    }
    pubGoal(pos);
    geometry_msgs::PoseStamped goal;
    hfn_->getGoal(&goal);

    Eigen::Vector2f goal_pos(goal.pose.position.x, goal.pose.position.y);
    if ((pos -  goal_pos).norm() < 1.0)
    {
      cmd.linear.x = std::sqrt(vel(0) *  vel(0)  + vel(1) *  vel(1));
      if(ifMovingBack){
          cmd.linear.x = -cmd.linear.x;
      }
      cmd.angular.z = (init_yaw - cur_yaw > 0) ? params_.d_yaw : -params_.d_yaw;
    }

    /// @songhao directly assign traj vel to cmd_vel_linear
    cmd.linear.x = std::sqrt(vel(0) *  vel(0)  + vel(1) *  vel(1));
    if(ifMovingBack){
        cmd.linear.x = -cmd.linear.x;
    }
    cmd.angular.z = (init_yaw - cur_yaw > 0) ? params_.d_yaw : -params_.d_yaw;
    //cout << "cmd.linear.x = " << cmd.linear.x << endl;
  }
  else{
    //std::cout << "the traj mode is " << params_.traj_mode << std::endl;
    bool valid_waypoint = updateWaypoint();

    if (!valid_waypoint) {
      stop();
      callback_(UNREACHABLE);
      return;
    }
    hfn_->getCommandVel(&cmd);
  }


//  // check if enabling turning
//  if (!turning_ &&
//      linear_distance(pose_.pose, goals_.back().pose) < 0.8*params_.goal_tol) {
//    turning_ = 0.0 <= params_.goal_tol_ang && params_.goal_tol_ang <= M_PI;
//    if (turning_) {
//      ROS_INFO("Ignoring HFN and turning instead");
//    }
//  }
//  if (turning_) {
//    cmd.linear.x = 0.0;
//    double diff =
//      angles::shortest_angular_distance(tf::getYaw(pose_.pose.orientation),
//                                        tf::getYaw(goals_.back().pose.orientation));
//    double speed = hfn_->params().w_max;
//    if (fabs(diff) < M_PI / 8.0) {
//      speed /= 1.5;
//    }
//    cmd.angular.z = copysign(speed, diff);
//  }

  vel_pub_.publish(cmd);

}

void HFNWrapper::onOdom(const nav_msgs::Odometry &odom) {
  flags_.have_odom = true;
  hfn_->setOdom(odom);
  cur_linear_vel_ = odom.twist.twist.linear.x;
  cur_pos_(0) = odom.pose.pose.position.x;
  cur_pos_(1) = odom.pose.pose.position.y;
  if(!ifSetGoal){
      return;
  }
  if(!ifHasInitOrien) {
      ifHasInitOrien = true;
//      init_orient_.w() = odom.pose.pose.orientation.w;
//      init_orient_.x() = odom.pose.pose.orientation.x;
//      init_orient_.y() = odom.pose.pose.orientation.y;
//      init_orient_.z() = odom.pose.pose.orientation.z;
      double x0 = odom.pose.pose.position.x;
      double y0 = odom.pose.pose.position.y;
      double xg = fixGoal.pose.position.x;
      double yg = fixGoal.pose.position.y;
      init_yaw = atan2(yg - y0, xg - x0);
//      init_yaw = atan2(2.0*(init_orient_.w()*init_orient_.z() + init_orient_.x()*init_orient_.y()),
//                       -1.0 + 2.0 * (init_orient_.w() * init_orient_.w() + init_orient_.x() * init_orient_.x()));

  }else{
      cur_orient_.w() = odom.pose.pose.orientation.w;
      cur_orient_.x() = odom.pose.pose.orientation.x;
      cur_orient_.y() = odom.pose.pose.orientation.y;
      cur_orient_.z() = odom.pose.pose.orientation.z;
      cur_yaw = atan2(2.0*(cur_orient_.w()*cur_orient_.z() + cur_orient_.x()*cur_orient_.y()),
                       -1.0 + 2.0 * (cur_orient_.w() * cur_orient_.w() + cur_orient_.x() * cur_orient_.x()));
  }
  //ROS_INFO("init yaw = %f, cur_yaw = %f", init_yaw, cur_yaw);
}

void HFNWrapper::setGoal(const vector<geometry_msgs::PoseStamped> &p) {
  ROS_INFO("HFNWrapper: Got final goal: (%.2f, %.2f, %.2f)",
           p.back().pose.position.x, p.back().pose.position.y, p.back().pose.position.z);
  
  if (!initialized()) {
    ROS_WARN("HFNWrapper: NOTREADY (Haven't received: %s)",
             uninitializedString().c_str());
    callback_(NOTREADY);
    return;
  }

  if(!ifSetGoal){
      ifSetGoal = true;
      oneGoal = p[0];
      fixGoal = oneGoal;
      start.pose.position.x = cur_pos_(0);
      start.pose.position.y = cur_pos_(1);
      start.pose.position.z = 0.0;
  }

  for (int i = 0; i < p.size(); ++i) {
    const geometry_msgs::PoseStamped &pose = p.at(i);
    double x = pose.pose.position.x, y = pose.pose.position.y;
    if (map_->getCell(x, y) == NULL) {
      ROS_WARN("HFNWrapper: UNREACHABLE (Goal %f %f is outside map limits)", x, y);
      callback_(UNREACHABLE);
      return;
    }
  }

  // goal and p is close, do not replan
  // if (goals_.size() > 0 &&
  //     linear_distance(goals_.back().pose, p.back().pose) < params_.goal_tol) {
  //   ROS_INFO("HFNWrapper: Goal is close, not replanning");
  //   return;
  // }
  goals_ = p;
  pose_history_.clear();
  goal_time_ = ros::Time::now();
  // If we were turning to orient towards the last goal, and we're still pretty
  // close to goal, don't bother moving closer, just keep on turning
  turning_ = (turning_ &&
              linear_distance(goals_.back().pose, pose_.pose) < 2 * params_.goal_tol);

  // Plan path from current location to the final location in goals_,
  // passing through all intermediate points in goals_
  scarab::Path path;
  path.push_back(Eigen::Vector2f(pose_.pose.position.x, pose_.pose.position.y));
  for (std::vector<geometry_msgs::PoseStamped>::iterator it = goals_.begin();
       it != goals_.end(); ++it) {
    // Check if goals_ location is reachable
    if (map_->getCell(it->pose.position.x, it->pose.position.y)->occ_dist <
        params_.lethal_occ_dist) {

      double startx = it->pose.position.x, starty = it->pose.position.y;
      double newx, newy;
      bool valid = map_->nearestPoint(startx, starty, params_.lethal_occ_dist,
                                      &newx, &newy);
      if (valid) {
        ROS_WARN("HFNWrapper: Adjusted goal at %f %f", startx, starty);
        it->pose.position.x = newx;
        it->pose.position.y = newy;
      } else {
        ROS_WARN("HFNWrapper: UNREACHABLE (Goal at (%f, %f) is too close to obstacle)",
                 it->pose.position.x, it->pose.position.y);
        stop();
        callback_(UNREACHABLE);
        return;
      }
    }
    // Plan a path to goals_ location
    geometry_msgs::Pose last_pose;
    std::cout << "the path.back() is " << path.back()<< std::endl;
    Eigen::Vector2f cur_pos(0, 0);
    double cur_time = (ros::Time::now() - traj_start_time_).toSec();
    
    //if the class is valid, use the cur_pos`
    if (!cur_traj_.empty()){
      cur_traj_.getPosition(cur_time, cur_pos);
      std::cout << "the cur_pos is " << cur_pos(0) << " " << cur_pos(1) << std::endl;
    }

    // if the distance of cur_pos and path back is not large, use cur_pos
    double dist = std::sqrt((cur_pos(0) - path.back()(0))*(cur_pos(0) - path.back()(0)) +
                  (cur_pos(1) - path.back()(1))*(cur_pos(1) - path.back()(1)));
    if (dist < 10 * params_.waypoint_spacing) {
      last_pose.position.x = cur_pos(0);
      last_pose.position.y = cur_pos(1);
    } else {
      last_pose.position.x = path.back()(0);
      last_pose.position.y = path.back()(1);
    }


    if (linear_distance(last_pose, it->pose) > params_.waypoint_spacing) {
      scarab::Path path_segment =
        map_->astar(last_pose.position.x, last_pose.position.y,
                    it->pose.position.x, it->pose.position.y,
                    params_.lethal_occ_dist, params_.allow_unknown_path);
      if (path_segment.size() != 0) {
        for (size_t i=0; i<path_segment.size(); ++i) {
          path.push_back(path_segment[i]);
        }
      } else {
        ROS_WARN("HFNWrapper: UNREACHABLE (No path found to goal)");
        stop();
        callback_(UNREACHABLE);
        return;
      }
    } else {
      path.push_back(Eigen::Vector2f(it->pose.position.x, it->pose.position.y));
    }
  }

  // Generate evenly spaced path

  if (params_.traj_mode == 0)
  {
    waypoints_.clear();
    waypoints_.push_back(path[0]);

    std::cout << "path size: " << path.size() << std::endl;
    for (size_t i = 0; i < path.size() - 1; ++i) {
      const Eigen::Vector2f& prev = waypoints_.back();
      const Eigen::Vector2f& curr = path[i];
      if ((prev - curr).norm() > params_.waypoint_spacing) {
        waypoints_.push_back(path[i-1]);
      }
    }
    waypoints_.push_back(path.back());

    std::cout << "waypoints size: " << waypoints_.size() << std::endl;
    pubWaypoints();
  }else if (params_.traj_mode == 1)
  {
    waypoints_.clear();
    waypoint_times_.clear();
    
    waypoints_.push_back(path[0]);

    for (size_t i = 1; i < path.size() -1 ; ++i) {
      const Eigen::Vector2f& prev = waypoints_.back();
      const Eigen::Vector2f& curr = path[i];
      if ((prev - curr).norm() > 5 * params_.waypoint_spacing) {
        //std::cout << "path[" << i << "]: " << path[i] << std::endl;
        waypoints_.push_back(path[i]);
      }
    }
    waypoints_.push_back(path.back());
    //std::cout << " waypoints_ size: " << waypoints_.size() << std::endl;
    

    Eigen::Vector2f start_vel(0, 0), start_acc(0, 0);
    double cur_time = (ros::Time::now() - traj_start_time_).toSec();
    if(!cur_traj_.empty())
    {
      cur_traj_.getVelocity(cur_time, start_vel);
      cur_traj_.getAcceleration(cur_time, start_acc);
    }
    std::cout << "start vel is " << start_vel(0) << " " << start_vel(1) << std::endl;
    std::cout << "start acc is " << start_acc(0) << " " << start_acc(1) << std::endl;

    waypoints_.clear();
    Eigen::Vector2f start_pos(start.pose.position.x, start.pose.position.y);
    Eigen::Vector2f goal_pos(oneGoal.pose.position.x, oneGoal.pose.position.y);
    waypoints_.push_back(start_pos);
    waypoints_.push_back(goal_pos);
    waypoint_times_.clear();

    //gen_traj(path[0], start_vel, start_acc);
    gen_traj(start_pos, start_vel, start_acc);
    pubWaypoints();
    pubTraj();
  }

  timeout_timer_ = nh_.createTimer(ros::Duration(params_.timeout),
                                   &HFNWrapper::timeout,
                                   this, true);
  active_ = true;
}

void HFNWrapper::gen_traj(Eigen::Vector2f &xi, 
                          Eigen::Vector2f &vi, 
                          Eigen::Vector2f &ai)
{ 
  std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> ic;
  ic.push_back(vi);
  ic.push_back(ai);
  traj_gen_->clearWaypoints();
  traj_gen_->setInitialConditions(xi, ic);
  traj_gen_->addWaypoints(waypoints_);
  
  std::vector<float> waypoint_times;
  waypoint_times.reserve(waypoints_.size());
  if(waypoint_times_.size() == 0)
  {
    waypoint_times = traj_gen_->computeTimesTrapezoidSpeed(params_.max_speed, params_.max_acc);
  }
  else
  {
    waypoint_times.push_back(0);  // Time for the current state
    for(const auto &t : waypoint_times_)
      waypoint_times.push_back(t);
  }

  traj_gen_->calculate(waypoint_times);
  float max_jerk_des = 100;
  prev_traj_ = cur_traj_;

  Trajectory traj  = traj_gen_->optimizeWaypointTimes(params_.max_speed, params_.max_acc, max_jerk_des);

  std::cout << "the traj.empty() size is " << traj.empty() << std::endl;

  cur_traj_ = traj;
  traj_start_time_ = ros::Time::now();  //////// get sim time from 0

  //////// get time from system clock: for planner
  send_traj_start_time = ros::Time(ros::WallTime::now().toSec());

  return;
}

void HFNWrapper::stop() {
  ROS_INFO("HFNWrapper: Stopping");
  active_ = false;

  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  vel_pub_.publish(cmd_vel);

  timeout_timer_.stop();

  waypoints_.clear();
  waypoint_times_.clear();
  pubWaypoints();
  traj_gen_->clearWaypoints();
  //hasGoal = false;
}

void HFNWrapper::move_back() {
    if(!ifMovingBack){
        ifMovingBack = true;
    }else{
        ifMovingBack = false;
    }
    ROS_INFO("Move back !");
    active_ = true;

    geometry_msgs::PoseStamped temp;
    temp = oneGoal;
    oneGoal = start;
    goals_.clear();
    goals_.push_back(oneGoal);
    start = temp;
    setGoal(goals_);
}

void HFNWrapper::timeout(const ros::TimerEvent &event) {
  ROS_WARN("HFNWrapper: TIMEOUT (Didn't get to goal in time)");
  stop();
  callback_(TIMEOUT);
}

// work for traj_mode 0
bool HFNWrapper::updateWaypoint() {
  // Direct robot towards successor of point that it is closest to
  float min_dist = numeric_limits<float>::infinity();
  int min_ind = -1;
  // Get closest visible waypoint
  Eigen::Vector2f pos(pose_.pose.position.x, pose_.pose.position.y);
  for (size_t wayind = 0; wayind < waypoints_.size(); ++wayind) {
    if (!map_->lineOfSight(pos.x(), pos.y(),
                           waypoints_[wayind].x(), waypoints_[wayind].y(),
                           params_.los_margin, params_.allow_unknown_los)) {
      continue;
    }
    float dist = (pos - waypoints_[wayind]).squaredNorm();
    if (dist < min_dist) {
      min_dist = dist;
      min_ind = wayind;
    }
  }

  int ind_delta = 0;
  if (min_ind == -1) {
    return false;
  } else {
    while (ind_delta < 20 &&
           static_cast<unsigned>(min_ind + 1) < waypoints_.size() &&
           map_->lineOfSight(pos.x(), pos.y(),
                             waypoints_[min_ind+1].x(), waypoints_[min_ind+1].y(),
                             params_.los_margin, params_.allow_unknown_los)) {
      ++min_ind;
      ++ind_delta;
    }

    Eigen::Vector2f pos;
    pos(0) =  waypoints_[min_ind].x();
    pos(1) =  waypoints_[min_ind].y();
    pubGoal(pos);
    return true;

  }
}


void HFNWrapper::pubGoal(Eigen::Vector2f &pos)
{

  geometry_msgs::PoseStamped goal;
  goal.header.stamp = ros::Time::now();
  goal.header.frame_id = params_.map_frame;
  goal.pose.position.x = pos(0);
  goal.pose.position.y = pos(1);
  goal.pose.position.z = goals_.back().pose.position.z;

  hfn_->setGoal(goal);

  visualization_msgs::Marker m;
  m.header.stamp = ros::Time();
  m.header.frame_id = params_.map_frame;
  m.action = visualization_msgs::Marker::ADD;
  m.type = visualization_msgs::Marker::SPHERE;
  m.id = 100;
  m.ns = params_.name_space + "/waypoint";
  m.pose.orientation.w = 1.0;
  m.scale.x = 0.1;
  m.scale.y = 0.1;
  m.scale.z = 0.1;
  m.color.a = 1.0;
  m.color.r = 1.0;
  m.pose.position = goal.pose.position;
  vis_pub_.publish(m);

  hfn_->getGoal(&goal);
  m.id += 1;
  m.header.frame_id = goal.header.frame_id;
  m.color.r = 0.0;
  m.color.b = 1.0;
  m.pose = goal.pose;
  vis_pub_.publish(m);
  return;
}

void HFNWrapper::pubWaypoints() {
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = params_.map_frame;

  nav_msgs::Path ros_path;
  ros_path.header = header;
  ros_path.poses.resize(waypoints_.size());
  for (size_t i = 0; i < waypoints_.size(); ++i) {
    ros_path.poses[i].header = header;
    tf::poseTFToMsg(tf::Pose(tf::createIdentityQuaternion(),
                             tf::Vector3(waypoints_[i].x(), waypoints_[i].y(), 0)),
                    ros_path.poses[i].pose);
  }
  path_pub_.publish(ros_path);
}

void HFNWrapper::pubTraj()
{
  
  visualization_msgs::Marker sphere, line_strip;
  sphere.header.frame_id = line_strip.header.frame_id = params_.map_frame;
  sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
  sphere.type = visualization_msgs::Marker::SPHERE_LIST;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
  sphere.id = 0;
  line_strip.id = 1000;

  sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
  sphere.color.r = line_strip.color.r = 0.0;
  sphere.color.g = line_strip.color.g = 0.2;
  sphere.color.b = line_strip.color.b = 0.7;
  sphere.color.a = line_strip.color.a = 1.0;
  sphere.scale.x = 0.1;
  sphere.scale.y = 0.1;
  sphere.scale.z = 0.1;
  line_strip.scale.x = 0.05;
  geometry_msgs::Point pt;
  Eigen::Vector2f pos, vel;
  double total_time = cur_traj_.getTotalTime();
  for (double t = 0; t < total_time; t += 0.5)
  {
    cur_traj_.getPosition(t, pos);
    pt.x = pos(0);
    pt.y = pos(1);
    pt.z = 0.1;
    //cur_traj_.getVelocity(t, vel);
    
    //std::cout << "the pos is " << pos.transpose()<< std::endl;
    sphere.points.push_back(pt);
    line_strip.points.push_back(pt);
  }

  traj_pub_.publish(sphere);
  traj_pub_.publish(line_strip);

  static int traj_id = 0;

  //publish the poly trajectory by coefficients

  /// agent is defined as scarabxx, and we want the xx number as int
  std::string agent = params_.agent;
  int agent_num = std::stoi(agent.substr(6, 2));


  kr_traj_msgs::PolyTrajByCoeffs poly_traj;
  poly_traj.header.stamp      =  ros::Time(ros::WallTime::now().toSec());
  //poly_traj.header.seq = 1;
  poly_traj.header.frame_id = "simulator";

  poly_traj.total_agent_num = 1;  ///  ......
  //poly_traj.start_time      = traj_start_time_;
  poly_traj.start_time      =   send_traj_start_time;
  poly_traj.agent_id        = params_.robot_id;
  poly_traj.traj_id         = traj_id;
  poly_traj.order           = 3;
  poly_traj.dim             = 2;
  poly_traj.agent_radius    = params_.agent_r;

  int piece_num = cur_traj_.getPieceNum();
  poly_traj.coeff_x.resize(piece_num * 4);
  poly_traj.coeff_y.resize(piece_num * 4);
  poly_traj.duration.resize(piece_num);

  for (int i = 0; i < piece_num; ++i)
  {
    Eigen::MatrixX2f coeff = cur_traj_.getCoeffs(i);
    cout << coeff << endl;
    for (int j = 0; j < 4; ++j)
    {
      poly_traj.coeff_x[i * 4 + j] = coeff(j, 0);
      poly_traj.coeff_y[i * 4 + j] = coeff(j, 1);
    }

    //std::cout << "the duration is " << cur_traj_.getPieceTime(i) << std::endl;
    poly_traj.duration[i] = cur_traj_.getPieceTime(i);
  }

  poly_pub_.publish(poly_traj);


  traj_id++;
  ROS_INFO("[scarab]: send the trajectory to vicon_map, with stamp(current time) = %f, and frame_id = %s",
           poly_traj.header.stamp.toSec(), poly_traj.header.frame_id.c_str());
}




//=============================== MoveServer ================================//

MoveServer::MoveServer(const string &server_name, HFNWrapper *wrapper) :
  pnh_("~"), wrapper_(wrapper), action_name_(ros::names::resolve(server_name)),
  as_(nh_, server_name, false) {

  pnh_.param("stop_on_preempt", stop_on_preempt_, true);
  wrapper->registerStatusCallback(boost::bind(&MoveServer::hfnCallback, this, _1));

  as_.registerGoalCallback(boost::bind(&MoveServer::goalCallback, this));
  as_.registerPreemptCallback(boost::bind(&MoveServer::preemptCallback, this));
}

void MoveServer::preemptCallback() {
  ROS_INFO("Preempted");
  if (stop_on_preempt_) {
    wrapper_->stop();
  }
  as_.setPreempted();
}

void MoveServer::goalCallback() {
  scarab_msgs::MoveGoalConstPtr goal = as_.acceptNewGoal();
  if (!goal->stop) {
    if (goal->target_poses.empty()) {
      ROS_WARN("HFN was sent empty set of poses");
      wrapper_->stop();
      scarab_msgs::MoveResult result;
      result.final_status = scarab_msgs::MoveResult::UNREACHABLE;
      as_.setAborted(result);
    } else {
      ROS_INFO("%s got request to (%.2f, %.2f, %.2f)",
               action_name_.c_str(),
               goal->target_poses.back().pose.position.x,
               goal->target_poses.back().pose.position.y,
               goal->target_poses.back().pose.position.z);
      ROS_INFO("goal->target_poses size() = %d", goal->target_poses.size());
      wrapper_->setGoal(goal->target_poses);
    }
  } else {
    ROS_INFO("%s got request to stop", action_name_.c_str());
    wrapper_->stop();
    scarab_msgs::MoveResult result;
    result.final_status = scarab_msgs::MoveResult::FINISHED;
    as_.setSucceeded(result);
  }
}

void MoveServer::hfnCallback(HFNWrapper::Status status) {
  scarab_msgs::MoveResult result;
  if (status == HFNWrapper::FINISHED) {
    result.final_status = scarab_msgs::MoveResult::FINISHED;
    as_.setSucceeded(result);
  } else {
    std::string statstr;
    switch (status) {
      case HFNWrapper::TIMEOUT:
        statstr = "TIMEOUT";
        result.final_status = scarab_msgs::MoveResult::TIMEOUT;
        break;
      case HFNWrapper::STUCK:
        statstr = "STUCK";
        result.final_status = scarab_msgs::MoveResult::STUCK;
        break;
      case HFNWrapper::NOTREADY:
        statstr = "NOTREADY";
        result.final_status = scarab_msgs::MoveResult::NOTREADY;
        break;
      case HFNWrapper::UNREACHABLE:
        statstr = "UNREACHABLE";
        result.final_status = scarab_msgs::MoveResult::UNREACHABLE;
        break;
      default:
        ROS_ERROR("%s Unknown status %d", action_name_.c_str(), status);
    }
    ROS_WARN("%s Aborting (status=%s)", action_name_.c_str(), statstr.c_str());
    as_.setAborted(result);
  }
}

} // end namespace scarab
