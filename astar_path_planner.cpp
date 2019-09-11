#include <vector>
#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>

#include "astar_path_planner/occupancy_grid.h"
#include "astar_path_planner/node.h"
#include "astar_path_planner/open_set.h"
#include "astar_path_planner/closed_set.h"
#include "astar_path_planner/marker.h"

#include "astar_path_planner/PlanPath.h"

namespace
{
double heuristicCost(astar_path_planner::WorldPosition a, astar_path_planner::WorldPosition b)
{
  // Return a heuristic cost between two world positions
  
  // YOUR CODE HERE
  
  double dx = b.x - a.x;
  double dy = b.y - a.y;
  double heuristic_cost  = sqrt ( pow( dx , 2) + pow( dy , 2) );

  return  heuristic_cost; 
}

void waitForKey()
{
  ROS_INFO("Paused, press enter to continue...");
  std::cin.get();
}
}  // namespace
namespace astar_path_planner
{
class PathPlanner
{
public:
  explicit PathPlanner(ros::NodeHandle& nh);

private:
  double inflation_radius_ = 0.2;
  double plan_path_main_loop_rate_ = 500;  // The target rate of the main loop in planPath()
  ros::Duration plan_path_main_loop_duration_ = ros::Duration(1. / plan_path_main_loop_rate_);

  OccupancyGrid occupancy_grid_;

  WorldPosition start_position_{};
  WorldPosition goal_position_{};
  bool start_position_valid_ = false;
  bool goal_position_valid_ = false;

  ros::Subscriber initialpose_sub_{}, goal_sub_{};
  ros::Publisher inflated_map_pub_{}, start_pub_{}, goal_pub_{}, open_set_pub_{}, closed_set_pub_{}, path_pub_{};

  ros::ServiceServer plan_path_srv_{};

  void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose_msg);
  void goalCallback(const geometry_msgs::PoseStamped& pose_msg);
  visualization_msgs::Marker createSetMarker(const std::vector<Node>& nodes, MarkerColour colour);
  bool planPath(astar_path_planner::PlanPath::Request& req, astar_path_planner::PlanPath::Response& res);
};

PathPlanner::PathPlanner(ros::NodeHandle& nh)
{
  // Get parameters (a variable will not be changed if the ROS parameter has not been set)
  ros::param::get("~inflation_radius", inflation_radius_);
  ros::param::get("~plan_path_main_loop_rate", plan_path_main_loop_rate_);
  plan_path_main_loop_duration_ = ros::Duration(1. / plan_path_main_loop_rate_);

  // Wait for static_map to be available
  ROS_INFO("Waiting for static_map service...");
  ros::service::waitForService("static_map");

  // Get the map
  nav_msgs::GetMap get_map{};

  if (!ros::service::call("static_map", get_map))
  {
    ROS_ERROR("Unable to get map");
    ros::shutdown();
  }
  else
  {
    //map_ = get_map.response.map;
    ROS_INFO("Map received");
  }

  // Create an occupancy grid from the occupancy grid message
  occupancy_grid_ = OccupancyGrid(get_map.response.map, inflation_radius_);

  // Publish the inflated map
  inflated_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("map_inflated", 1, true);
  inflated_map_pub_.publish(occupancy_grid_.getOccupancyGridMsg());

  // Advertise marker publishers
  start_pub_ = nh.advertise<visualization_msgs::Marker>("path_planner/start", 1, true);
  goal_pub_ = nh.advertise<visualization_msgs::Marker>("path_planner/goal", 1, true);
  open_set_pub_ = nh.advertise<visualization_msgs::Marker>("path_planner/open_set", 1);
  closed_set_pub_ = nh.advertise<visualization_msgs::Marker>("path_planner/closed_set", 1);
  path_pub_ = nh.advertise<visualization_msgs::Marker>("path_planner/path", 1);

  // Subscribe to start and goal poses
  initialpose_sub_ = nh.subscribe("initialpose", 1, &PathPlanner::initialPoseCallback, this);
  goal_sub_ = nh.subscribe("move_base_simple/goal", 1, &PathPlanner::goalCallback, this);

  // Advertise PlanPath service server
  plan_path_srv_ = nh.advertiseService("plan_path", &PathPlanner::planPath, this);
}

void PathPlanner::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose_msg)
{
  // Check if the position is out of the map
  if (occupancy_grid_.isOutOfBounds(WorldPosition{ pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y }))
  {
    ROS_INFO("Start position is out of bounds");
    return;
  }

  // Check if the position is occupied
  if (occupancy_grid_.isOccupied(WorldPosition{ pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y }))
  {
    ROS_INFO("Start position occupied");
    return;
  }

  start_position_.x = pose_msg.pose.pose.position.x;
  start_position_.y = pose_msg.pose.pose.position.y;
  start_position_valid_ = true;

  // Clear path marker
  path_pub_.publish(createDeleteMarker());

  // Publish start marker
  start_pub_.publish(createSphereMarker(start_position_.x, start_position_.y, MarkerColour::GREEN, 0.1));
}

void PathPlanner::goalCallback(const geometry_msgs::PoseStamped& pose_msg)
{
  // Check if the position is out of the map
  if (occupancy_grid_.isOutOfBounds(WorldPosition{ pose_msg.pose.position.x, pose_msg.pose.position.y }))
  {
    ROS_INFO("Goal position is out of bounds");
    return;
  }

  // Check if the position is occupied
  if (occupancy_grid_.isOccupied(WorldPosition{ pose_msg.pose.position.x, pose_msg.pose.position.y }))
  {
    ROS_INFO("Goal position occupied");
    return;
  }

  goal_position_.x = pose_msg.pose.position.x;
  goal_position_.y = pose_msg.pose.position.y;
  goal_position_valid_ = true;

  // Clear path marker
  path_pub_.publish(createDeleteMarker());

  // Publish goal marker
  goal_pub_.publish(createSphereMarker(goal_position_.x, goal_position_.y, MarkerColour::RED, 0.1));
}

visualization_msgs::Marker PathPlanner::createSetMarker(const std::vector<Node>& nodes, MarkerColour colour)
{
  std::vector<WorldPosition> world_positions{};

  world_positions.reserve(nodes.size());

  for (const auto& n : nodes)
  {
    world_positions.emplace_back(occupancy_grid_.getWorldPosition(n.id));
  }

  return createSphereListMarker(world_positions, colour, 0.05);
}

bool PathPlanner::planPath(astar_path_planner::PlanPath::Request& req, astar_path_planner::PlanPath::Response& res)
{
  // Only try to plan a path if the start and goal are valid
  if (!start_position_valid_ || !goal_position_valid_)
  {
    ROS_ERROR("Start and/or goal positions not set");
    return false;
  }

  // Delete existing path markers
  path_pub_.publish(createDeleteMarker());

  Cell start_cell = occupancy_grid_.getCell(start_position_);
  Cell goal_cell = occupancy_grid_.getCell(goal_position_);

  OpenSet open_set{};
  ClosedSet closed_set{};

  Node start_node{};

  start_node.id = start_cell.id;
  // start_node has no parent

  start_node.cost = 0.;
  start_node.heuristic_cost = heuristicCost(start_position_, goal_position_);

  open_set.push(start_node);

  bool goal_found = false;

  while (ros::ok() && !open_set.empty())
  {
    ros::Time loop_start_time = ros::Time::now();

    // Remove the best node from "open_set" with "pop(req.heuristic_cost_weight)"
    // Put it on closed_set
    // If the node is the goal node, set "goal_found" to true and exit the loop (with "break;")
    // Get the cell adjacent to "id" with "occupancy_grid_.getAdjacentCells(id, req.diagonal_movement)"
    // For each adjacent cell:
    //     If the adjacent cell is already on the closed set, do nothing
    //     If the adjacent cell is already on the open set, update it if it's better (use the "update" method)
    //     If the adjacent cell is not on the closed or open sets, add it to the open set

    // YOUR CODE HERE
    
    ROS_INFO_STREAM(open_set);

    //best node with the lowest heur cost has been removed into lowest_n
    Node lowest_n = open_set.pop(req.heuristic_cost_weight);

    //print it out for test 
    ROS_INFO_STREAM("\n\nNode: \n" << lowest_n );  

    //put it into the closed set 
    closed_set.push(lowest_n);
    

    //check to see if node is the goal if yes break; 
    //create a while loop that will break once the goal node is found
    while (!goal_found)

    {
	 std::vector<AdjacentCell> neighbours_cells{}; 
	  
	 neighbours_cells = occupancy_grid_.getAdjacentCells(lowest_n.id, req.diagonal_movement);

	 //iterate though neighbours to get appropro info
	Node neighbour_node{}; 

	for (int x = 0 ; x < (neighbours_cells.size()) ; x++)
	{
	    //create a node using adjacent info 
	    
	 //   neighbour_node[x].id = neighbours_cells.id;
	 //   neighbour_node[x].parent_id  = start_node.id ; 
	 //   neighbour_node[x].cost  ;
	 //   neighbour_node[x].heuristic_cost  

	    //apropriate checks on the node
	    	  

	}
	//get the adjacent cells to the lowest code node	
	  

     //goal has been found
     goal_found = true ;
    }
    
    
    
    //if no: 


    //call adjacent cell

  
    
    // YOU DON'T NEED TO MODIFY ANYTHING AFTER THIS LINE

    // Publish the sets
    open_set_pub_.publish(createSetMarker(open_set.getNodes(), MarkerColour::YELLOW));
    closed_set_pub_.publish(createSetMarker(closed_set.getNodes(), MarkerColour::FUCHSIA));

    // Sleep if the duration of the loop is less than the target duration
    ros::Duration loop_duration = ros::Time::now() - loop_start_time;

    if (loop_duration < plan_path_main_loop_duration_)
    {
      (plan_path_main_loop_duration_ - loop_duration).sleep();
    }
  }

  if (!goal_found)
  {
    ROS_ERROR("No path to goal found");
    return false;
  }

  // Get the path from the closed set
  std::vector<int> path_ids = closed_set.getPath(start_cell.id, goal_cell.id);

  // Convert node IDs to world positions
  std::vector<WorldPosition> path_world_positions{};
  path_world_positions.reserve(path_ids.size());

  for (const auto id : path_ids)
  {
    path_world_positions.emplace_back(occupancy_grid_.getWorldPosition(id));
  }

  // Delete open and closed set markers
  ros::Duration(0.5).sleep();
  open_set_pub_.publish(createDeleteMarker());
  closed_set_pub_.publish(createDeleteMarker());

  // Publish a marker array for the path
  path_pub_.publish(createSphereListMarker(path_world_positions, MarkerColour::BLUE, 0.05));

  res.length_of_path = path_ids.size();
  res.number_of_nodes_in_closed_set = closed_set.size();

  return true;
}

}  // namespace astar_path_planner

int main(int argc, char** argv)
{
  ros::init(argc, argv, "particle_filter_localisation");

  ros::NodeHandle nh{};

  astar_path_planner::PathPlanner path_planner(nh);

  ros::spin();

  return 0;
}
