
#include "ros/ros.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

/*
* The SVG defines the even–odd rule by saying:
* This rule determines the "insideness" of a point on the canvas by drawing a ray
* from that point to infinity in any direction and counting the number of path
* segments from the given shape that the ray crosses. If this number is odd, the
* point is inside; if even, the point is outside.
*/
bool pnpoly(const float& x, const float& y, const geometry_msgs::Polygon& poly) {
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

    std::string target_frame_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tf2_;
    sensor_msgs::LaserScan::ConstPtr laser_scan_ptr_;

    ros::Subscriber laserscan_subscriber_;
    ros::Subscriber footprint_polygon_subscriber_;

    ros::Publisher footprint_pub_;
    ros::Publisher is_obstructed_pub_;

    void laserScanCallBack(const sensor_msgs::LaserScan::ConstPtr& msg);
    void footprintCallBack(const geometry_msgs::PolygonStamped::ConstPtr& msg);
    bool checkScanIn2DFootPrint(const geometry_msgs::PolygonStamped& poly);
    bool checkScanIn2DFootPrint(const geometry_msgs::Polygon& poly);
};

LaserScanObstacleObserver::LaserScanObstacleObserver(ros::NodeHandle& nh, ros::NodeHandle& pnh): 
    nh_(nh), pnh_(pnh), tf2_(buffer_),  target_frame_("base_link") {
    //pnh_.param("costmap_sources", topics_string_, std::string("map1 map2"));

    footprint_pub_ = pnh_.advertise<geometry_msgs::PolygonStamped>("new_poly", 1000, true);
    is_obstructed_pub_ = pnh_.advertise<std_msgs::Bool>("is_obstructed", 1000, true);

    laserscan_subscriber_ = nh_.subscribe("scan", 1, &LaserScanObstacleObserver::laserScanCallBack, this);
    footprint_polygon_subscriber_ = nh_.subscribe("move_base/global_costmap/footprint", 1, &LaserScanObstacleObserver::footprintCallBack, this);
  }

void LaserScanObstacleObserver::laserScanCallBack(const sensor_msgs::LaserScan::ConstPtr& msg) {
    target_frame_ = msg->header.frame_id;

    laser_scan_ptr_ = msg;
}

void LaserScanObstacleObserver::footprintCallBack(const geometry_msgs::PolygonStamped::ConstPtr& msg) {
    std_msgs::Bool is_obstructed_msg;
    geometry_msgs::TransformStamped map_to_base_link_tf;
    geometry_msgs::PolygonStamped polygon_out;
    polygon_out.polygon.points.resize(msg->polygon.points.size());
    polygon_out.header = msg->header;
    polygon_out.header.frame_id = target_frame_;

    try {
        map_to_base_link_tf = buffer_.lookupTransform(target_frame_, msg->header.frame_id, ros::Time(0), ros::Duration(0.2));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
      return;
    }

    // I have to transform them one by one because, to my knowledge, the default tf2
    // library does not support one shot transformation of geometry_msgs::Polygon
    try {
        for (size_t i = 0; i < msg->polygon.points.size(); i++) {
            geometry_msgs::Point t_in, t_out;
            t_in.x = msg->polygon.points[i].x;
            t_in.y = msg->polygon.points[i].y;
            t_in.z = msg->polygon.points[i].z;
            tf2::doTransform<geometry_msgs::Point>(t_in, t_out, map_to_base_link_tf);
            polygon_out.polygon.points[i].x = t_out.x;
            polygon_out.polygon.points[i].y = t_out.y;
            polygon_out.polygon.points[i].z = t_out.z;
        }
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
      return;
    }

    if (checkScanIn2DFootPrint(polygon_out)) {
        is_obstructed_msg.data = true;
    } else {
        is_obstructed_msg.data = false;
    }

    is_obstructed_pub_.publish(is_obstructed_msg);
    footprint_pub_.publish(polygon_out);
}

bool LaserScanObstacleObserver::checkScanIn2DFootPrint(const geometry_msgs::PolygonStamped& poly) {
    bool scan_point_in_polygon = false;
    float curr_ang = laser_scan_ptr_->angle_min;

    for (std::vector<float>::const_iterator it = laser_scan_ptr_->ranges.begin(); it != laser_scan_ptr_->ranges.end(); it++) {
        scan_point_in_polygon = pnpoly(*it*cos(curr_ang), *it*sin(curr_ang), poly.polygon);
        curr_ang += laser_scan_ptr_->angle_increment;
        if (scan_point_in_polygon) {
            return true;
        }
    }
    return false;
}

bool LaserScanObstacleObserver::checkScanIn2DFootPrint(const geometry_msgs::Polygon& poly) {
    bool scan_point_in_polygon = false;
    float curr_ang = laser_scan_ptr_->angle_min;

    for (std::vector<float>::const_iterator it = laser_scan_ptr_->ranges.begin(); it != laser_scan_ptr_->ranges.end(); it++) {
        scan_point_in_polygon = pnpoly(*it*cos(curr_ang), *it*sin(curr_ang), poly);
        curr_ang += laser_scan_ptr_->angle_increment;
        if (scan_point_in_polygon) {
            return true;
        }
    }
    return false;
}

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
