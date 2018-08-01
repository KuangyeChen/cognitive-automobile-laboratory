#pragma once

#include <boost/geometry/algorithms/num_points.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Path.h>


namespace utils_ros {

//! \note copies only points of outer polygon to the message,
//!       because geometry_msgs/Polygon does not support inner polygons
template <typename Point, int PointDimension>
geometry_msgs::PolygonStamped toMsg(const std_msgs::HeaderConstPtr& header,
                                    const boost::geometry::model::polygon<Point>& polygon) {
    namespace bg = boost::geometry;

    geometry_msgs::PolygonStamped msg;
    geometry_msgs::Point32 point;

    msg.header.frame_id = header->frame_id;
    msg.header.stamp = header->stamp;

    for (int i = 0; i < (int)bg::num_points(polygon.outer()); ++i) {
        point.x = bg::get<0>(polygon.outer()[i]);
        point.y = bg::get<1>(polygon.outer()[i]);
#if PointDimension > 2
        msg.polygon.points.back().z = bg::get<2>(polygon.outer()[i]);
#endif
        msg.polygon.points.push_back(point);
    }

    return msg;
}

template <typename Point, int PointDimension>
nav_msgs::Path toMsg(const std_msgs::HeaderConstPtr& header,
                     const boost::geometry::model::linestring<Point>& linestring) {
    namespace bg = boost::geometry;

    nav_msgs::Path msg;
    msg.header.frame_id = header->frame_id;
    msg.header.stamp = header->stamp;
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = header->frame_id;
    pose_msg.header.stamp = header->stamp;
    pose_msg.pose.orientation.w = 1.;

    for (int i = 0; i < (int)bg::num_points(linestring); ++i) {
        pose_msg.pose.position.x = bg::get<0>(linestring[i]);
        pose_msg.pose.position.y = bg::get<1>(linestring[i]);
#if PointDimension > 2
        pose_msg.pose.position.z = bg::get<2>(linestring[i]);
#endif
        msg.poses.push_back(pose_msg);
    }

    return msg;
}
} // namespace utils_ros
