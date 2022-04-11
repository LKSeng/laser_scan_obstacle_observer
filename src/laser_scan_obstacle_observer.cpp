#include "laser_scan_obstacle_observer/laser_scan_obstacle_observer.h"

LaserScanObstacleObserver::LaserScanObstacleObserver(ros::NodeHandle& nh, ros::NodeHandle& pnh): 
    nh_(nh), pnh_(pnh), tf2_(buffer_) {
    dynamic_config_callback_ = boost::bind(&LaserScanObstacleObserver::dynamicReconfigureCallBack, this, _1, _2);
    dynamic_config_server_.setCallback(dynamic_config_callback_);

    pnh_.param<float>("footprint_padding", footprint_padding_, 0.0);

    footprint_pub_ = pnh_.advertise<geometry_msgs::PolygonStamped>("footprint", 1000, true);
    is_obstructed_pub_ = pnh_.advertise<std_msgs::Bool>("is_obstructed", 1000, true);

    laserscan_subscriber_ = nh_.subscribe("scan", 1, &LaserScanObstacleObserver::laserScanCallBack, this);
    footprint_polygon_subscriber_ = nh_.subscribe("move_base/global_costmap/footprint", 1, &LaserScanObstacleObserver::footprintCallBack, this);

    publish_when_ready_ = nh.createTimer(ros::Rate(10.0), &LaserScanObstacleObserver::timerCheckReadyCallBack, this, false, true);
    publish_updates_ = nh.createTimer(ros::Rate(10.0), &LaserScanObstacleObserver::timerPublishUpdateCallBack, this, false, false);
  }

void LaserScanObstacleObserver::dynamicReconfigureCallBack(laser_scan_obstacle_observer::LaserScanObstacleObserverConfig &config, uint32_t level) {
    footprint_padding_ = config.footprint_padding;
}

void LaserScanObstacleObserver::timerCheckReadyCallBack(const ros::TimerEvent& event) {
    if (!footprint_polygon_ptr_) {
        ROS_WARN_THROTTLE(2.0, "Footprint not published yet!");
    }

    if (!laser_scan_ptr_) {
        ROS_WARN_THROTTLE(2.0, "Scan was not published yet!");
    }

    // start if both are received at least once
    if (laser_scan_ptr_ and footprint_polygon_ptr_) {
        publish_updates_.start();
        publish_when_ready_.stop();
    }
}

void LaserScanObstacleObserver::timerPublishUpdateCallBack(const ros::TimerEvent& event) {
    if (ros::Time::now() - footprint_polygon_ptr_->header.stamp > ros::Duration(1.0)) {
        ROS_WARN_THROTTLE(2.0, "Footprint was not updated for more than 1s");
    }

    if (ros::Time::now() - laser_scan_ptr_->header.stamp > ros::Duration(1.0)) {
        ROS_WARN_THROTTLE(2.0, "Scan was not updated for more than 1s");
    }

    std_msgs::Bool is_obstructed_msg;
    geometry_msgs::TransformStamped map_to_base_link_tf;
    geometry_msgs::PolygonStamped polygon_out;
    polygon_out.polygon.points.resize(footprint_polygon_ptr_->polygon.points.size());
    polygon_out.header = footprint_polygon_ptr_->header;
    polygon_out.header.frame_id = laser_scan_ptr_->header.frame_id;

    try {
        map_to_base_link_tf = buffer_.lookupTransform(laser_scan_ptr_->header.frame_id, footprint_polygon_ptr_->header.frame_id, ros::Time(0), ros::Duration(0.2));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
      return;
    }

    // I have to transform them one by one because, to my knowledge, the default tf2
    // library does not support one shot transformation of geometry_msgs::Polygon
    try {
        for (size_t i = 0; i < footprint_polygon_ptr_->polygon.points.size(); i++) {
            geometry_msgs::Point t_in, t_out;
            t_in.x = footprint_polygon_ptr_->polygon.points[i].x;
            t_in.y = footprint_polygon_ptr_->polygon.points[i].y;
            t_in.z = footprint_polygon_ptr_->polygon.points[i].z;
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

    padFootprintPolygon(polygon_out.polygon);

    if (checkScanIn2DFootprint(polygon_out)) {
        is_obstructed_msg.data = true;
    } else {
        is_obstructed_msg.data = false;
    }

    is_obstructed_pub_.publish(is_obstructed_msg);
    footprint_pub_.publish(polygon_out);
}

void LaserScanObstacleObserver::laserScanCallBack(const sensor_msgs::LaserScan::ConstPtr& msg) {
    laser_scan_ptr_ = msg;
}

void LaserScanObstacleObserver::footprintCallBack(const geometry_msgs::PolygonStamped::ConstPtr& msg) {
    footprint_polygon_ptr_ = msg;
}

void LaserScanObstacleObserver::padFootprintPolygon(geometry_msgs::Polygon& poly) {
    // pad footprint in place
    for (unsigned int i = 0; i < poly.points.size(); i++)
    {
        poly.points[i].x += sign0(poly.points[i].x) * footprint_padding_;
        poly.points[i].y += sign0(poly.points[i].y) * footprint_padding_;
    }
}

bool LaserScanObstacleObserver::checkScanIn2DFootprint(const geometry_msgs::PolygonStamped& poly_stamped) {
    bool scan_point_in_polygon = false;
    float curr_ang = laser_scan_ptr_->angle_min;

    for (std::vector<float>::const_iterator it = laser_scan_ptr_->ranges.begin(); it != laser_scan_ptr_->ranges.end(); it++) {
        scan_point_in_polygon = pnpoly(*it*cos(curr_ang), *it*sin(curr_ang), poly_stamped.polygon);
        curr_ang += laser_scan_ptr_->angle_increment;
        if (scan_point_in_polygon) {
            return true;
        }
    }
    return false;
}

bool LaserScanObstacleObserver::checkScanIn2DFootprint(const geometry_msgs::Polygon& poly) {
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
