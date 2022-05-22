#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/String.h"

nav_msgs::OccupancyGrid map_ground, map_first, map_out;
std_msgs::String mux_no;

void mapGroundCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map_ground.header = msg->header;
    map_ground.info = msg->info;
    map_ground.data = msg->data;
}

void mapFirstCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map_first.header = msg->header;
    map_first.info = msg->info;
    map_first.data = msg->data;
}

void muxCallback(const std_msgs::String::ConstPtr& msg)
{
    mux_no.data = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_multiplexer");
    ros::NodeHandle nh;
    ros::Subscriber map_ground_sub = nh.subscribe("map_0", 1, mapGroundCallback);
    ros::Subscriber map_first_sub = nh.subscribe("map_1", 1, mapFirstCallback);
    ros::Subscriber mux_sub = nh.subscribe("mux", 1, muxCallback);
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        while (1)
        {
            if (mux_no.data == "1")
            {
                map_out.header = map_first.header;
                map_out.info = map_first.info;
                map_out.data = map_first.data;
            }
            else
            {
                map_out.header = map_ground.header;
                map_out.info = map_ground.info;
                map_out.data = map_ground.data;
            }
            map_pub.publish(map_out);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    return 0;
}