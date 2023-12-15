#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

// ============================ (A) Callbacks for Subscribers ============================
double goal_reached = NAN;
void plannerCallback(const std_msgs::Float64MultiArray::ConstPtr msg)
{
  goal_reached = msg->data[2];
}

std::vector<float> scan_ranges;
void scanCallback(const sensor_msgs::LaserScan::ConstPtr msg)
{
  scan_ranges = msg->ranges;
}

double pos_x, pos_y, heading = NAN;
void odomCallback(const nav_msgs::Odometry::ConstPtr msg)
{ // ground truth in simulation for turtlebot
  double qx = msg->pose.pose.orientation.x;
  double qy = msg->pose.pose.orientation.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;

  pos_x = msg->pose.pose.position.x;
  pos_y = msg->pose.pose.position.y;
  heading = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qz * qz + qy * qy));
}

// ============================ (B) Main Function ============================
int main(int argc, char **argv)
{
  // init a ros node
  ros::init(argc, argv, "range_detect_node");
  ros::NodeHandle nh;

  // ------------------------  (1) LOAD SUBSCRIBERS / PUBLISHERS and MESSAGES ------------------------
  ros::Subscriber sub_scan = nh.subscribe("/scan", 1, &scanCallback);
  ros::Subscriber sub_odom = nh.subscribe("/odom", 1, &odomCallback);
  ros::Subscriber sub_planner = nh.subscribe<std_msgs::Float64MultiArray>("/planner", 1, plannerCallback);
  ros::Publisher pub_obs = nh.advertise<std_msgs::Float64MultiArray>("/obstacles", 1);
  std_msgs::Float64MultiArray msg_obs;
  msg_obs.data.resize(6);

  // ------------------------  (2) WAIT UNTIL MESSAGE RECEIVED ------------------------
  // goal_reached and heading were initialised to NAN, before messages are received.
  // Otherwise, garbage (uninitialised) values in target_x, target_y, pos_x, pos_y, and the former two, are used in the while loop.
  ROS_INFO("range_detect_node waiting for messages");
  while (ros::ok() && (std::isnan(goal_reached) || std::isnan(heading) || scan_ranges.empty()))
  {
    ros::spinOnce(); // update subscriber buffers
  }
  ROS_INFO("range_detect_node initialized successfully");

  // ------------------------  (3) MAIN LOOP ------------------------
  int north_idx, west_idx, south_idx, east_idx, rotate, northwest_idx, northeast_idx;
  ros::Rate rate(50);
  while (ros::ok() && goal_reached == 0)
  {
    ros::spinOnce(); // update subscriber buffers

    // === (a) FIND THE RANGES BY ROTATING FROM ROBOT FRAME TO WORLD FRAME ===
    // scan_ranges is 360 long, 1 degree resolution, in robot frame of reference
    north_idx = 0;
    west_idx = 90;
    south_idx = 180;
    east_idx = 270;
    northwest_idx = 20;
    northeast_idx = 340;
    rotate = heading / M_PI * 180; // convert to degree, -pi <= heading < pi

    // get the idx corresponding to world's north, west, south and east
    // can use for loops, but use no loops for easier learning
    north_idx -= rotate;
    west_idx -= rotate;
    south_idx -= rotate;
    east_idx -= rotate;
    northwest_idx -= rotate;
    northeast_idx -= rotate;
    // constrain to 0 and 359 inclusive -- remember arrays can only be accessed between 0 and (array size - 1)
    if (north_idx < 0)
      north_idx += 360;
    else if (north_idx > 359)
      north_idx -= 360;
    if (west_idx < 0)
      west_idx += 360;
    else if (west_idx > 359)
      west_idx -= 360;
    if (south_idx < 0)
      south_idx += 360;
    else if (south_idx > 359)
      south_idx -= 360;
    if (east_idx < 0)
      east_idx += 360;
    else if (east_idx > 359)
      east_idx -= 360;
    if (northwest_idx < 0)
      northwest_idx += 360;
    else if (northwest_idx > 359)
      northwest_idx -= 360;
    if (northeast_idx < 0)
      northeast_idx += 360;
    else if (northeast_idx > 359)
      northeast_idx -= 360;

    // === (b) PUBLISH SCAN RANGES ===
    msg_obs.data[0] = scan_ranges[north_idx];
    msg_obs.data[1] = scan_ranges[west_idx];
    msg_obs.data[2] = scan_ranges[south_idx];
    msg_obs.data[3] = scan_ranges[east_idx];
    msg_obs.data[4] = scan_ranges[northwest_idx];
    msg_obs.data[5] = scan_ranges[northeast_idx];
    pub_obs.publish(msg_obs);

    // === (d) CONTROL LOOP RATE (OPTIONAL HERE) ===
    rate.sleep();
  }

  return 0;
}
