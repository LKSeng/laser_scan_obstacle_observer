#include "laser_scan_obstacle_observer/laser_scan_obstacle_observer.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_scan_obstacle_observer_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ROS_INFO("This node; online");

    LaserScanObstacleObserver observer_node(nh, pnh);

    ros::spin();

    return 0;
}

