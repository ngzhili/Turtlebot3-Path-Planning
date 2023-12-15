/*
  Name: NG ZHILI
*/

#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <deque>

const int MAP_MAX_X = 10; // size of map in x-axis
const int MAP_MAX_Y = 10; // size of map in y-axis

struct Cell
{
  bool walls[4] = {false}; // defaulted to no walls
  bool north_deadend = false; // default deadend is false
};
Cell cells[MAP_MAX_X][MAP_MAX_Y]; // initialise structure variable cells is defined by structure type Cell.

// ============================ (A) Callbacks for Subscribers ============================
double pos_x, pos_y = NAN; // Initialise pos_x and pos_y as NAN
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) // Callback for receiving odometry data
{ // ground truth in simulation for turtlebot
  pos_x = msg->pose.pose.position.x; // get turtlebot x coordinate in simulation
  pos_y = msg->pose.pose.position.y; // get turtlebot y coordinate in simulation
}

double north_scan_range, west_scan_range, south_scan_range, east_scan_range, northwest_scan_range, northeast_scan_range = NAN;
void rangesCallback(const std_msgs::Float64MultiArray::ConstPtr &msg) // Callback for receiving range data
{
  north_scan_range = msg->data[0];
  west_scan_range = msg->data[1];
  south_scan_range = msg->data[2];
  east_scan_range = msg->data[3];
  northwest_scan_range = msg->data[4];
  northeast_scan_range = msg->data[5];
}

std::vector<std::array<int, 2>> findPath(int idx_x, int idx_y, int goal_x, int goal_y)
{
  // using Breadth First Search (BFS) a.k.a. Flood Fill, but without filling every cell in the map
  // BFS uses stack based priority queue
  // This BFS uses 4-connected grids, so costs are chebyshev distances from the start

  std::vector<std::array<int, 2>> path;

  // ====================== (A) initialise BFS nodes ===============================
  struct Node
  {
    int x = -1, y = -1;     // default values
    int cost = 10000;       // initalise cost's default value as very large number
    Node *parent = nullptr; // default no parent,  parent pointer variable points to null pointer value
  };
  Node nodes[MAP_MAX_X][MAP_MAX_Y]; // contains information specific for BFS for each cell
  for (int x = 0; x < MAP_MAX_X; ++x) // initialise map nodes: (0,0), (0,1), (1,0) ..., (9,9)
  {
    for (int y = 0; y < MAP_MAX_Y; ++y)
    {
      nodes[x][y].x = x;
      nodes[x][y].y = y;
    }
  }

  // ===================== (B) initialise start and push to queue =====================
  nodes[idx_x][idx_y].cost = 0;          // set cost (or step) of start (robot index) to zero
  std::deque<Node *> queue;              // x, y, cost (or step). std::list is better for this, but kept deque for beginners (can use [] operator to examine)  
  queue.push_back(&nodes[idx_x][idx_y]); // push start to queue

  // ===================== (C) Loop until goal found or queue empty =====================

  while (!queue.empty()) // When queue is empty, break out of loop (goal not found)
  {
    // ---------------------- (1) Poll the queue -------------------------

    // ENTER YOUR COMMENT HERE // Refer to report for more detailed explaination of comments
    Node *cur_node = queue.front(); //  visit node at front of the queue
    queue.pop_front();              //  remove visited node from front of queue
    int cur_x = cur_node->x;        // get current node's x value        
    int cur_y = cur_node->y;        // get current node's y value
    int cur_cost = cur_node->cost;  // get current node's cost

    // ---------------------- (2) Find the path if goal is found, and break loop -------------------------
    if (cur_x == goal_x && cur_y == goal_y) // check if goal has been reached when current node's x,y coordinates is same as the goal node's x,y coordinates
    {
      Node *node = cur_node;        // assign cur_node to node variable
      do
      {
        path.push_back({node->x, node->y}); // add node into BFS found path
        node = node->parent;                // set node to it's parent node
      } while (node != nullptr);            // while loop to backtrace node to find BFS path. Break out of loop when node has no parent (node's parent is null pointer)
      break;
    }
    // END OF YOUR COMMENT HERE

    // ---------------------- (3) Search neighbors and queue them if cheaper -------------------------
    // should use for loops, but kept it for beginners to understand
    Cell *cur_cell = &cells[cur_x][cur_y]; // point to current cell
        
    if (cur_x <= 5 && cur_y <= 1) {
      
      // === (a) check NORTH neighbor ===
      if (!cur_cell->walls[0] && cur_x < MAP_MAX_X - 1)
      {                                              // north wall does not exist AND north cell exists ==> north cell is accessible
        Node *north_node = &nodes[cur_x + 1][cur_y]; // point to current node
        int new_north_cell_cost = cur_cost + 1;
        int old_north_cell_cost = north_node->cost;
        if (old_north_cell_cost > new_north_cell_cost)
        { // north cell is cheaper to get there from current cell
          // update parent and cost information for north cell, and queue it
          north_node->cost = new_north_cell_cost;
          north_node->parent = cur_node;
          queue.push_back(north_node);
        }
      }

      // === (b) check WEST neighbor ===
      if (!cur_cell->walls[1] && cur_y < MAP_MAX_Y - 1)
      {                                             // west wall does not exist AND west cell exists ==> west cell is accessible
        Node *west_node = &nodes[cur_x][cur_y + 1]; // point to current node
        int new_west_cell_cost = cur_cost + 1;
        int old_west_cell_cost = west_node->cost;
        if (old_west_cell_cost > new_west_cell_cost)
        { // west cell is cheaper to get there from current cell
          // update parent and cost information for west cell, and queue it
          west_node->cost = new_west_cell_cost;
          west_node->parent = cur_node;
          queue.push_back(west_node);
        }
      } 

      // === (c) check SOUTH neighbor ===
      if (!cur_cell->walls[2] && cur_x > 0)
      {                                              // south wall does not exist AND south cell exists ==> south cell is accessible
        Node *south_node = &nodes[cur_x - 1][cur_y]; // point to current node
        int new_south_cell_cost = cur_cost + 1;
        int old_south_cell_cost = south_node->cost;
        if (old_south_cell_cost > new_south_cell_cost)
        { // south cell is cheaper to get there from current cell
          // update parent and cost information for south cell, and queue it
          south_node->cost = new_south_cell_cost;
          south_node->parent = cur_node;
          queue.push_back(south_node);
        }
      }

      // === (d) check EAST neighbor ===
      if (!cur_cell->walls[3] && cur_y > 0)
      {                                             // east wall does not exist AND east cell exists ==> east cell is accessible
        Node *east_node = &nodes[cur_x][cur_y - 1]; // point to current node
        int new_east_cell_cost = cur_cost + 1;
        int old_east_cell_cost = east_node->cost;
        if (old_east_cell_cost > new_east_cell_cost)
        { // east cell is cheaper to get there from current cell
          // update parent and cost information for east cell, and queue it
          east_node->cost = new_east_cell_cost;
          east_node->parent = cur_node;
          queue.push_back(east_node);
        }
      }
    }
    else
    { 
      // === (b) check WEST neighbor ===
      if (!cur_cell->walls[1] && cur_y < MAP_MAX_Y - 1)
      {                                             // west wall does not exist AND west cell exists ==> west cell is accessible
        Node *west_node = &nodes[cur_x][cur_y + 1]; // point to current node
        int new_west_cell_cost = cur_cost + 1;
        int old_west_cell_cost = west_node->cost;
        if (old_west_cell_cost > new_west_cell_cost)
        { // west cell is cheaper to get there from current cell
          // update parent and cost information for west cell, and queue it
          west_node->cost = new_west_cell_cost;
          west_node->parent = cur_node;
          queue.push_back(west_node);
        }
      }
      // === (c) check SOUTH neighbor ===
      if (!cur_cell->walls[2] && cur_x > 0)
      {                                              // south wall does not exist AND south cell exists ==> south cell is accessible
        Node *south_node = &nodes[cur_x - 1][cur_y]; // point to current node
        int new_south_cell_cost = cur_cost + 1;
        int old_south_cell_cost = south_node->cost;
        if (old_south_cell_cost > new_south_cell_cost)
        { // south cell is cheaper to get there from current cell
          // update parent and cost information for south cell, and queue it
          south_node->cost = new_south_cell_cost;
          south_node->parent = cur_node;
          queue.push_back(south_node);
        }
      }
      // === (a) check NORTH neighbor ===
      if (!cur_cell->walls[0] && cur_x < MAP_MAX_X - 1)
      {                                              // north wall does not exist AND north cell exists ==> north cell is accessible
        Node *north_node = &nodes[cur_x + 1][cur_y]; // point to current node
        int new_north_cell_cost = cur_cost + 1;
        int old_north_cell_cost = north_node->cost;
        if (old_north_cell_cost > new_north_cell_cost)
        { // north cell is cheaper to get there from current cell
          // update parent and cost information for north cell, and queue it
          north_node->cost = new_north_cell_cost;
          north_node->parent = cur_node;
          queue.push_back(north_node);
        }
      }
      // === (d) check EAST neighbor ===
      if (!cur_cell->walls[3] && cur_y > 0)
      {                                             // east wall does not exist AND east cell exists ==> east cell is accessible
        Node *east_node = &nodes[cur_x][cur_y - 1]; // point to current node
        int new_east_cell_cost = cur_cost + 1;
        int old_east_cell_cost = east_node->cost;
        if (old_east_cell_cost > new_east_cell_cost)
        { // east cell is cheaper to get there from current cell
          // update parent and cost information for east cell, and queue it
          east_node->cost = new_east_cell_cost;
          east_node->parent = cur_node;
          queue.push_back(east_node);
        }
      }
    }
  }
  return path;
}

void fillWalls(int idx_x, int idx_y, std::string &dbg_str, int idx_goal_x, int idx_goal_y)
{
  dbg_str += "\tWalls: ";
  // === (ENCHANCEMENT) check for north adjacent cell dead ends ===
  // check if there is a north wall detected in the north adjacent cell
  bool north2_has_wall =  ((1.7 <= north_scan_range) && (north_scan_range <= 2));
  // check if there is a west wall detected in the north adjacent cell
  bool northwest_has_wall = ((1.15 <= northwest_scan_range) && (northwest_scan_range <= 1.75));
  // check if there is a east wall detected in the north adjacent cell
  bool northeast_has_wall = ((1.15 <= northeast_scan_range) && (northeast_scan_range <= 1.75));
  // check if north adjacent cell is detected as a dead end
  bool north_is_deadend = (north2_has_wall && northwest_has_wall && northeast_has_wall);
  
  // check if north adjacent cell is detected as a dead end AND the north adjacent cell is not the destination cell
  if (north_is_deadend && !(idx_x == idx_goal_x && idx_y == idx_goal_y)){ 
      dbg_str += "NN(" + std::string(north2_has_wall ? "yes" : " no") + ", dist=" + std::to_string(north_scan_range) + "), ";
      dbg_str += "NW(" + std::string(northwest_has_wall ? "yes" : " no") + ", dist=" + std::to_string(northwest_scan_range) + "), ";
      dbg_str += "NE(" + std::string(northeast_has_wall ? "yes" : " no") + ", dist=" + std::to_string(northeast_scan_range) + "), ";
      
      cells[idx_x][idx_y].walls[0] = north_is_deadend;       // set current cell's north wall as occupied
      if (idx_x < MAP_MAX_X - 1)                            // if not at northmost part of the map
        cells[idx_x + 1][idx_y].walls[2] = north_is_deadend; // set north cell's south wall as occupied
      
      cells[idx_x][idx_y].north_deadend = north_is_deadend; // set cuurent cell's north_deadend to true
      dbg_str += "DEADEND("+ std::to_string(idx_x+1) + "," + std::to_string(idx_y) + "), ";
    }

  // walls are along grid lines, not center of grid cells
  // world's north is always in world's +x direction
  bool north_has_wall = north_scan_range <= 1;
  if (!cells[idx_x][idx_y].north_deadend){ // if north adjacent cell is not a deadend, we continue to update the current cell's north wall status
    cells[idx_x][idx_y].walls[0] = north_has_wall;       // set current cell's north wall as occupied
    if (idx_x < MAP_MAX_X - 1)                           // if not at northmost part of the map
      cells[idx_x + 1][idx_y].walls[2] = north_has_wall;} // set north cell's south wall as occupied
  dbg_str += "N(" + std::string(cells[idx_x][idx_y].walls[0] ? "yes" : " no") + ", dist=" + std::to_string(north_scan_range) + "), ";

  // world's west is always in world's +y direction
  bool west_has_wall = west_scan_range <= 1;
  cells[idx_x][idx_y].walls[1] = west_has_wall;       // set current cell's west wall as occupied
  if (idx_y < MAP_MAX_Y - 1)                          // if not at westmost part of the map
    cells[idx_x][idx_y + 1].walls[3] = west_has_wall; // set west cell's east wall as occupied
  dbg_str += "W(" + std::string(west_has_wall ? "yes" : " no") + ", dist=" + std::to_string(west_scan_range) + "), ";

  // world's south is always in world's -x direction
  bool south_has_wall = south_scan_range <= 1;
  cells[idx_x][idx_y].walls[2] = south_has_wall;       // set current cell's south wall as occupied
  if (idx_x > 1)                                       // if not at southmost part of the map
    cells[idx_x - 1][idx_y].walls[0] = south_has_wall; // set south cell's north wall as occupied
  dbg_str += "S(" + std::string(south_has_wall ? "yes" : " no") + ", dist=" + std::to_string(south_scan_range) + "), ";

  // world's east is always in world's -y direction
  bool east_has_wall = east_scan_range <= 1;
  cells[idx_x][idx_y].walls[3] = east_has_wall;       // set current cell's east wall as occupied
  if (idx_y > 1)                                      // if not at eastmost part of the map
    cells[idx_x][idx_y - 1].walls[1] = east_has_wall; // set east cell's west wall as occupied
  dbg_str += "E(" + std::string(east_has_wall ? "yes" : " no")+ ", dist=" + std::to_string(east_scan_range) + ")";

  dbg_str += "\n";
}
int main(int argc, char **argv)
{

  ros::init(argc, argv, "path_plan_node");
  ros::NodeHandle nh;

  // ------------------------  (0) RESET GAZEBO WORLD (CANNOT BE USED FOR REAL TBOT) ---------------------------------------
  ros::ServiceClient gaz_reset_srv = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation");
  gaz_reset_srv.waitForExistence();
  std_srvs::Empty gaz_reset_msg = std_srvs::Empty();
  gaz_reset_srv.call(gaz_reset_msg);

  // ------------------------  (1) LOAD SUBSCRIBERS / PUBLISHERS and MESSAGES ------------------------
  ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>("/odom", 1, odomCallback);
  ros::Subscriber sub_ranges_sub = nh.subscribe<std_msgs::Float64MultiArray>("/obstacles", 1, rangesCallback);
  ros::Publisher pub_planner = nh.advertise<std_msgs::Float64MultiArray>("/planner", 1, true); // latch
  std_msgs::Float64MultiArray msg_planner;
  msg_planner.data.resize(3);

  // ------------------------  (2) SIGNAL OTHER NODES TO START ------------------------
  msg_planner.data[2] = 0; // set goal reached to 0
  pub_planner.publish(msg_planner);

  // ------------------------  (3) LOAD ROS PARAMS ------------------------
  double pos_goal_x, pos_goal_y;
  int idx_goal_x, idx_goal_y;
  if (!nh.getParam("goal_x", pos_goal_x)) 
  {
    ROS_ERROR("[PathPlanNode]: goal_x Load Error"); // if cannot get goal_x param
    msg_planner.data[2] = -1; // set goal reached to signal other nodes to shutdown
    pub_planner.publish(msg_planner);
    ros::Duration(2).sleep(); // sleep for other nodes to react
    ros::requestShutdown();
    return 1;
  }
  idx_goal_x = std::floor(pos_goal_x);
  if (!nh.getParam("goal_y", pos_goal_y))
  {
    ROS_ERROR("[PathPlanNode]: goal_y Load Error"); // if cannot get goal_y param
    msg_planner.data[2] = -1; // set goal reached to signal other nodes to shutdown
    pub_planner.publish(msg_planner);
    ros::Duration(2).sleep(); // sleep for other nodes to react
    ros::requestShutdown();
    return 1;
  }
  idx_goal_y = std::floor(pos_goal_y);

  // ------------------------  (4) WAIT UNTIL MESSAGE RECEIVED ------------------------
  // goal_reached and heading were initialised to NAN, before messages are received.
  // Otherwise, garbage (uninitialised) values in target_x, target_y, pos_x, pos_y, and the former two, are used in the while loop.
  ROS_INFO("[PathPlanNode]: waiting for messages");
  while (ros::ok() && (std::isnan(pos_y) || std::isnan(east_scan_range)))
  {                  // depends on odom only
    ros::spinOnce(); // update subscriber buffers
  }
  ROS_INFO("[PathPlanNode]: initialized successfully.");

  // ------------------------  (5) MAIN LOOP ------------------------
  ros::Rate rate(10);
  int idx_x, idx_y;
  bool north_has_wall, west_has_wall, south_has_wall, east_has_wall;
  double err_x, err_y;
  double target_x, target_y;
  std::string dbg_str;

  while (ros::ok())
  {
    ros::spinOnce(); // update subscriber buffers
    dbg_str = "-------- [PathPlanNode] -------------------- \n";

    // === (a) GET CELL INDEX WHERE ROBOT IS ===
    // cells are between grid lines, in center of grid cells
    idx_x = std::floor(pos_x); // round down to integer value (grid cell x coordinate)
    idx_y = std::floor(pos_y); // round down to integer value (grid cell y coordinate)
    dbg_str += "\tRobot Idx(" + std::to_string(idx_x) + ", " + std::to_string(idx_y) + ")\n";
    dbg_str += "\tRobot Pos(" + std::to_string(pos_x) + ", " + std::to_string(pos_y) + ")\n";

    // === (b) IGNORE PLANNER AND WALL UPDATES WHEN ROBOT IS OUT OF MAP ===
    if (idx_x < 0 || idx_x >= MAP_MAX_X || idx_y < 0 || idx_y >= MAP_MAX_Y) // check if robot is outside map boundaries
    {
      dbg_str += "\tRobot is outside map. Planner and wall updates suspended.";
      ROS_WARN_STREAM(dbg_str);
      rate.sleep();
      continue;
    }

    // === (c) IF GOAL IS REACHED, STOP OTHER NODES AND SHUTDOWN ===
    err_x = pos_goal_x - pos_x; // get linear error in x-axis between goal pos and current robot pos
    err_y = pos_goal_y - pos_y; // get linear error in y-axis between goal pos and current robot pos
    if (sqrt(err_x * err_x + err_y * err_y) < 0.2) // goal is reached if magnitude of displacement between goal pos and current robot pos is less than 0.2
    {
      dbg_str += "\tGOAL REACHED!";
      ROS_INFO_STREAM(dbg_str);

      // send message to shutdown other nodes
      msg_planner.data[2] = 1;
      pub_planner.publish(msg_planner);
      ros::Duration(2).sleep();
      break;
    }

    // === (d) OTHERWISE, UPDATE WALLS ON CURRENT AND NEIGHBOR (ADJACENT) CELLS ===
    fillWalls(idx_x, idx_y, dbg_str, idx_goal_x, idx_goal_y);

    // === (e) BFS FROM ROBOT POSITION TO GOAL ===
    std::vector<std::array<int, 2>> path = findPath(idx_x, idx_y, idx_goal_x, idx_goal_y); // path is {[goal_idx_x, goal_idx_y],[...],[pos_x, pos_y]}
    dbg_str += "\tBFS Path: ";
    for (auto &coord : path)
      dbg_str += "(" + std::to_string(coord[0]) + "," + std::to_string(coord[1]) + "), ";
    dbg_str += "\n";

    // === (f) PUBLISH NEXT CELL TARGET ===
    if (path.size() == 0)
    {
      //cannot find path
      // don't update target (use last)
    }
    else if (path.size() == 1)
    { // on goal cell
      target_x = pos_goal_x;
      target_y = pos_goal_y;
    }
    else
    {
      // get second last element, becos last element is current robot's cell idx
      target_x = path[path.size() - 2][0] + 0.5; // idx is floored, so need +0.5 to get to center of cell
      target_y = path[path.size() - 2][1] + 0.5;
    }
    dbg_str += "\tPublish Target: (" + std::to_string(target_x) + ", " + std::to_string(target_y) + ")";
    msg_planner.data[0] = target_x;
    msg_planner.data[1] = target_y;
    pub_planner.publish(msg_planner);
    ROS_INFO_STREAM(dbg_str);

    // === (g) CONTROL LOOP RATE (OPTIONAL HERE) ===
    rate.sleep();
  }

  return 0;
}