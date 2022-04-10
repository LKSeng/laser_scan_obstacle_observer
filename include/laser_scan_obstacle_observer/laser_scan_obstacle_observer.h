#ifndef LASER_SCAN_OBSTACLE_OBSERVER_H_
#define LASER_SCAN_OBSTACLE_OBSERVER_H_

#include "laser_scan_obstacle_observer/LaserScanObstacleObserverConfig.h"
#include "dynamic_reconfigure/server.h"

#include "ros/ros.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


/**
 * @brief Same as sign(x) but returns 0 if x is 0.
**/
inline double sign0(double x) {
  return x < 0.0 ? -1.0 : (x > 0.0 ? 1.0 : 0.0);
}

/**
 * @brief Checks if a point is in a polygon.
 * The SVG defines the even–odd rule by saying:
 * This rule determines the "insideness" of a point on the canvas by drawing a ray
 * from that point to infinity in any direction and counting the number of path
 * segments from the given shape that the ray crosses. If this number is odd, the
 * point is inside; if even, the point is outside.
**/
inline bool pnpoly(const float& x, const float& y, const geometry_msgs::Polygon& poly) {
    bool is_odd = false;

    // Hello me from the future! minus one was added because iterator.end() points to "end-element-plus-one"
    // and in particular, her member variables return 0 for x and y, messing up the calculations!!!
    // anyway this function would be called pnpoly now, it seems to be what people call it
    std::vector<geometry_msgs::Point32>::const_iterator prev_it = poly.points.end() - 1;

    for (std::vector<geometry_msgs::Point32>::const_iterator it = poly.points.begin(); it != poly.points.end(); it++) {
        // if ((x == it->x) and (y == it->y)) return true; // point is in corner
        if ((it->y > y) != (prev_it->y > y)) {
            if (x < (it->x + (prev_it->x - it->x)*(y - it->y)/(prev_it->y - it->y))) {
                is_odd = !is_odd;
            }
        }
        prev_it = it;
    }
    return is_odd;
}

class LaserScanObstacleObserver {
public:
    LaserScanObstacleObserver(ros::NodeHandle& nh, ros::NodeHandle& pnh);
protected:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    dynamic_reconfigure::Server<laser_scan_obstacle_observer::LaserScanObstacleObserverConfig> dynamic_config_server_;
    dynamic_reconfigure::Server<laser_scan_obstacle_observer::LaserScanObstacleObserverConfig>::CallbackType dynamic_config_callback_;

    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tf2_;
    sensor_msgs::LaserScan::ConstPtr laser_scan_ptr_;
    geometry_msgs::PolygonStamped::ConstPtr footprint_polygon_ptr_;
    float footprint_padding_;

    ros::Subscriber laserscan_subscriber_;
    ros::Subscriber footprint_polygon_subscriber_;

    ros::Publisher footprint_pub_;
    ros::Publisher is_obstructed_pub_;

    ros::Timer publish_when_ready_;
    ros::Timer publish_updates_;

    void dynamicReconfigureCallBack(laser_scan_obstacle_observer::LaserScanObstacleObserverConfig &config, uint32_t level);
    void timerCheckReadyCallBack(const ros::TimerEvent& event);
    void timerPublishUpdateCallBack(const ros::TimerEvent& event);
    void laserScanCallBack(const sensor_msgs::LaserScan::ConstPtr& msg);
    void footprintCallBack(const geometry_msgs::PolygonStamped::ConstPtr& msg);
    void padFootprintPolygon(geometry_msgs::Polygon& poly);
    bool checkScanIn2DFootprint(const geometry_msgs::PolygonStamped& poly_stamped);
    bool checkScanIn2DFootprint(const geometry_msgs::Polygon& poly);
};

#endif //LASER_SCAN_OBSTACLE_OBSERVER_H_
