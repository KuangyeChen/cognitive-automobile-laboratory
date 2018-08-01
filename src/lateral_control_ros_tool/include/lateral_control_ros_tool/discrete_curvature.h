#ifndef LATERAL_CONTROL_ROS_TOOL_DISCRETE_CURVATURE_H
#define LATERAL_CONTROL_ROS_TOOL_DISCRETE_CURVATURE_H
#include <Eigen/Dense>

namespace lateral_control_ros_tool {

/*!
 * \brief cross2 the two dimension cross product of two vectors.
 *  The cross product of (v1, v2) and (w1, w2) is be the 3rd coordinate of the cross product of (v1, v2, 0) and (w1, w2, 0).
 *  This the signed area of a parallelepiped spanned by the input vector.
 *  eigen might implement this in future versions: http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1037
 * \param a the first vector
 * \param b the second vector
 * \return the 2D cross-product of a and b.
 */
inline double cross2(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
  return a.x() * b.y() - b.x() * a.y();
}


/*!
 * \brief discreteCurvature calculates the approximate curvature between p1, p2 and p3.
 *  If your curve is given as a set of discrete, noise-free, and reasonably
 *	closely-spaced points, you can use the formula for a circumscribing circle
 * 	through each set of three successive points as an approximation to the
 *	curvature at the center point. The curvature of such a circle is equal to
 *	four times the area of the triangle formed by the three points, divided by
 *	the product of the triangle's three sides.
 * \param p1 the first point
 * \param p2 the second point
 * \param p3 the third point
 * \return the curvature
 */
inline double discreteCurvature(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& p3) {
  return 2.0 * cross2(p2 - p1, p3 - p2) /
      ((p2 - p1).norm() * (p3 - p2).norm() * (p1 - p3).norm());
}

} //namespace lateral_control_ros_tool

#endif // LATERAL_CONTROL_ROS_TOOL_DISCRETE_CURVATURE_H
