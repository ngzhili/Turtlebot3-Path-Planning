/*
  EE3305/ME3243
  Name: NG ZHILI
*/

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

const double MAX_LINEAR_VEL = 0.7;   // real life max is 2.2, sim can go faster.
const double MAX_ANGULAR_VEL = M_PI; // M_PI from math.h. Real max is higher, sim can go faster.

// ============================ (A) Callbacks for Subscribers ============================
double target_x, target_y, goal_reached = 0;
void plannerCallback(const std_msgs::Float64MultiArray::ConstPtr target_msg)
{
  target_x = target_msg->data[0];
  target_y = target_msg->data[1];
  goal_reached = target_msg->data[2];
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

  ros::init(argc, argv, "bot_control_node");
  ros::NodeHandle nh;

  // ------------------------  (1) LOAD ROS PARAMS ------------------------
  double Kp_a, Ki_a, Kd_a, Kp_x, Ki_x, Kd_x, dt;
  if (!nh.getParam("Kp_a", Kp_a))
  {
    ROS_ERROR("Kp_a Load Error");
    ros::requestShutdown();
    return 1;
  }
  if (!nh.getParam("Ki_a", Ki_a))
  {
    ROS_ERROR("Ki_a Load Error");
    ros::requestShutdown();
    return 1;
  }
  if (!nh.getParam("Kd_a", Kd_a))
  {
    ROS_ERROR("Kd_a Load Error");
    ros::requestShutdown();
    return 1;
  }
  if (!nh.getParam("Kp_x", Kp_x))
  {
    ROS_ERROR("Kp_x Load Error");
    ros::requestShutdown();
    return 1;
  }
  if (!nh.getParam("Ki_x", Ki_x))
  {
    ROS_ERROR("Ki_x Load Error");
    ros::requestShutdown();
    return 1;
  }
  if (!nh.getParam("Kd_x", Kd_x))
  {
    ROS_ERROR("Kd_x Load Error");
    ros::requestShutdown();
    return 1;
  }
  if (!nh.getParam("dt", dt))
  {
    ROS_ERROR("dt Load Error");
    ros::requestShutdown();
    return 1;
  }

  // ------------------------  (2) LOAD SUBSCRIBERS / PUBLISHERS and MESSAGES ------------------------
  ros::Subscriber sub_planner = nh.subscribe<std_msgs::Float64MultiArray>("/planner", 1, plannerCallback);
  ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>("/odom", 1, odomCallback);
  ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  geometry_msgs::Twist msg_cmd; // for cmd_vel (values are by default initialised to zero)

  // ------------------------  (3) WAIT UNTIL MESSAGE RECEIVED ------------------------
  // goal_reached and heading were initialised to NAN, before messages are received.
  // Otherwise, garbage (uninitialised) values in target_x, target_y, pos_x, pos_y, and the former two, are used in the while loop.
  ROS_INFO("bot_control_node waiting for messages");
  while (ros::ok() && std::isnan(heading))
  {
    ros::spinOnce(); // update subscriber buffers
  }
  ROS_INFO("bot_control_node initialized successfully.");

  // ------------------------ (4) PID LOOP ------------------------
  double error_pos = 0, error_pos_prev, error_heading_prev, error_heading;
  double error_x, error_y;
  double vel_x = 0, vel_heading = 0; // for sending to cmd_vel
  double target_heading;
  double I_angular, I_linear, D_angular, D_pos; // can be used for PID
  ros::Rate rate(1 / dt);                       // makes sure each iteration is at least dt long
  while (ros::ok() && goal_reached == 0) // run PID algorithm as long as goal is not reached
  {
    ros::spinOnce(); // update subscriber buffers

    // === (a) CALCULATE ERRORS ===
    error_pos_prev = error_pos; // Stores the previous error_pos in the error_pos_prev variable.
    error_heading_prev = error_heading; //Stores the previous error_heading in the error_heading_prev variable.

    error_x = target_x - pos_x; // get error along x-axis between current cell and destination cell (center of the cell)
    error_y = target_y - pos_y; // get error along y-axis between current and destination cell (center of the cell)
    error_pos = sqrt(error_x * error_x + error_y * error_y); // calculate error displacement between the current position of the Turtlebot3 and the target cell

    // the angle is with respect to the +x direction
    target_heading = atan2(error_y, error_x); // radians
    error_heading = target_heading - heading; // radians
    // make sure -pi <= error_heading < pi. If else statements can be used becos target_heading and heading are betw -pi and pi.
    if (error_heading < -M_PI) // M_PI is pi = 3.1415..., defined in math.h
      error_heading += 2 * M_PI;
    if (error_heading > M_PI)
      error_heading -= 2 * M_PI;

    // === (b) PID SUMS ===
    // ROS_INFO("[Bot Control Node]: %f,%f", error_x, error_y); // print example

    I_angular = dt * error_heading; // calculate the integral terms for angle by using the product of the time derivative (dt) and error_heading.
    I_linear = dt * error_pos; // calculate the integral terms for linear displacement by using the product of the time derivative (dt) and error_pos.
    D_angular = (error_heading_prev - error_heading) / dt;
    D_pos = (error_pos_prev - error_pos) / dt;

    // ENTER YOUR PID CODE HERE

    vel_heading = (Kp_a * error_heading) + (Ki_a * I_angular) + (Kd_a * D_angular); // calculate new angular velocity
    vel_x = (Kp_x * error_pos) + (Ki_x * I_linear) + (Kd_x * D_pos); // calculate new linear velocity

    // END OF YOUR PID CODE HERE

    // === (c) SATURATE ===
    if (vel_x > MAX_LINEAR_VEL) // restrict vel_x to maximum magnitude of MAX_LINEAR_VEL = 0.7
      vel_x = MAX_LINEAR_VEL;
    if (vel_x < -MAX_LINEAR_VEL)
      vel_x = -MAX_LINEAR_VEL;
    if (vel_heading > MAX_ANGULAR_VEL) // restrict vel_heading to maximum magnitude of MAX_ANGULAR_VEL = M_PI
      vel_heading = MAX_ANGULAR_VEL;
    if (vel_heading < -MAX_ANGULAR_VEL)
      vel_heading = -MAX_ANGULAR_VEL;

    // === (d) MOVE ROBOT ===
    msg_cmd.linear.x = vel_x;
    msg_cmd.angular.z = vel_heading;
    pub_cmd.publish(msg_cmd);

    // === (e) CONTROL LOOP RATE (REQUIRED TO MAKE SURE DT IS MAINTAINED) ===
    rate.sleep();
  }

  // Tell robot to stop after reaching the destination cell
  msg_cmd.linear.x = 0;
  msg_cmd.angular.z = 0;
  pub_cmd.publish(msg_cmd);

  ros::Duration(1.0).sleep();

  return 0;
}