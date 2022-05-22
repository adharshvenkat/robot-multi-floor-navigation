#include "ros/ros.h"
#include "std_msgs/String.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "math.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
tf::Transform odom_tf;
double robot_x, robot_y, robot_roll, robot_pitch, robot_yaw, inc_x, inc_y, angle_to_wp, distance_to_wp;
bool flag1 = true; bool flag2 = true; bool flag3 = true;
bool once1 = true; bool once2 = true; bool once3 = true;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom_tf.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    odom_tf.setRotation(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
    robot_x = (odom_tf.getOrigin()).getX();
    robot_y = (odom_tf.getOrigin()).getY();
    odom_tf.getBasis().getRPY(robot_roll, robot_pitch, robot_yaw);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_floor_nav_node");
    ros::NodeHandle nh;

    ros::Publisher elevator_pub = nh.advertise<std_msgs::String>("elevator", 1);
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Publisher mux_pub = nh.advertise<std_msgs::String>("mux", 1);
    ros::Subscriber odom_sub = nh.subscribe("/odometry/filtered", 1, odomCallback);
    std_msgs::String elevator_msg;
    geometry_msgs::Twist twist_msg;
    std_msgs::String mux_msg;
    geometry_msgs::Point wp;

    
    MoveBaseClient ac("move_base", true);
    move_base_msgs::MoveBaseGoal goal;

    while (!ac.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move base action server to come up");
    }

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        if (flag1)
        {
            if (once1)
            {
                mux_msg.data = "0";
                mux_pub.publish(mux_msg);
                once1 = false;
            }
            else
            {
                // Send the waypoint in front of the elevator to the navigation stack
                goal.target_pose.header.frame_id = "odom";
                goal.target_pose.pose.position.x = -1.7;
                goal.target_pose.pose.position.y = 4.5;
                goal.target_pose.pose.orientation.z = 1.0;

                ac.sendGoal(goal);

                ac.waitForResult();        

                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_INFO("Reached in front of the elevator on ground level");
                    flag1 = false;
                    continue;
                }
                else
                {
                    ROS_INFO("Trying again to reach goal...");
                }
            }
        }
        else
        {
            if (flag2)
            {
                if (once2)
                {
                    // Calling the elevator to the ground level
                    elevator_msg.data = "0";
                    elevator_pub.publish(elevator_msg);
                    ros::Duration(3).sleep();
                    once2 = false;
                    continue;
                }
                else
                {
                    // Using closed loop speed control to enter the elevator
                    wp.x = -3.8;
                    wp.y = 4.5;
                    inc_x = wp.x - robot_x;
                    inc_y = wp.y - robot_y;
                    angle_to_wp = atan2(inc_y, inc_x);
                    distance_to_wp = pow((pow(inc_x, 2) + pow(inc_y, 2)), 0.5);
                    if (std::abs(angle_to_wp - robot_yaw) > 0.26)
                    {
                        twist_msg.linear.x = 0.0;
                        twist_msg.angular.z = 0.9;
                    }
                    else if (distance_to_wp > 0.2)
                    {
                        twist_msg.linear.x = 1.0;
                        twist_msg.angular.z = 0.0;
                    }
                    else
                    {
                        twist_msg.linear.x = 0.0;
                        twist_msg.angular.z = 0.0;
                        flag2 = false;
                        continue;
                    }
                    twist_pub.publish(twist_msg);
                }
            }   
            else
            {
                if (flag3)
                {
                    if (once3)
                    {
                        // Wait for the elevator door to close, send call the elevator to first level, change the map to first level, and wait for thhe elevator door to open
                        ros::Duration(10).sleep();
                        elevator_msg.data = "1";
                        elevator_pub.publish(elevator_msg);
                        mux_msg.data = "1";
                        mux_pub.publish(mux_msg);
                        ros::Duration(6).sleep();
                        once3 = false;
                        continue;
                    }
                    else
                    {
                        // Navigate to a waypoint outside the elevator on first level using closed loop speed control
                        wp.x = -1.5;
                        wp.y = 4.5;
                        inc_x = wp.x - robot_x;
                        inc_y = wp.y - robot_y;
                        angle_to_wp = atan2(inc_y, inc_x);
                        distance_to_wp = pow((pow(inc_x, 2) + pow(inc_y, 2)), 0.5);
                        int direction = (angle_to_wp - robot_yaw)/std::abs(angle_to_wp - robot_yaw);
                        if (std::abs(angle_to_wp - robot_yaw) > 0.26)
                        {
                            twist_msg.linear.x = 0.0;
                            twist_msg.angular.z = 0.9;
                        }
                        else if (distance_to_wp > 0.2)
                        {
                            twist_msg.linear.x = 1.0;
                            twist_msg.angular.z = 0.0;
                        }
                        else
                        {
                            twist_msg.linear.x = 0.0;
                            twist_msg.angular.z = 0.0;
                            flag3 = false;
                            continue;
                        }
                        twist_pub.publish(twist_msg);
                    }
                }
                else
                {
                    // Send the final goal waypoint to the navgation stack
                    ros::Duration(1).sleep();
                    goal.target_pose.header.frame_id = "odom";
                    goal.target_pose.pose.position.x = 0.0;
                    goal.target_pose.pose.position.y = 10.0;
                    goal.target_pose.pose.orientation.z = 0.0;
                    goal.target_pose.pose.orientation.z = 1.0;

                    ac.sendGoal(goal);

                    ac.waitForResult();        

                    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    {
                        ROS_INFO("Reached final goal waypoint, shutting down robot :)");
                        break;
                    }
                    else
                    {
                        ROS_INFO("Trying again to reach goal...");
                    }
                }
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
   }    
   return 0;
}