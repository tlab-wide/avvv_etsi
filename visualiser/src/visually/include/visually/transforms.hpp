/**
 * @author AmirInt <amir.intelli@gmail.com>
 * @brief Includes the entities used in autonomous vehicle network visualisation
*/

#ifndef VISUALLY__TRANSFORMS_HPP_
#define VISUALLY__TRANSFORMS_HPP_

// C++
#include <cmath>

// ROS2
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// rviz_visual_tools
#include <rviz_visual_tools/rviz_visual_tools.hpp>

// GeographicLib
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/MGRS.hpp>


namespace transforms
{

/**
 * @brief Create the corresponding rviz_visual_tools::Colors
 * colour based on the given colour name
 * @param colour
 * @returns Enumerator corresponding to the colour name
 */
rviz_visual_tools::Colors getRvizColour(
    const std::string& colour);

/**
 * @brief Create geometry_msgs::msg::Pose base on the given
 * position and orientation values
 * @param position
 * @param orientation
 * @param pose
 */
void getGeometryPose(
    const double position[3]
    , const double orientation[3]
    , geometry_msgs::msg::Pose& pose);

/**
 * @brief Calculates the distance between two geometry_msgs::msg::Point's
 * @param point1
 * @param point2
 * @returns Calculated distance
 */
double dist(
    const geometry_msgs::msg::Point& point1
    , const geometry_msgs::msg::Point& point2);

/**
 * @brief Calculates the distance between two Eigen::Vector3d's
 * @param point1
 * @param point2
 * @returns Calculated distance
 */
double dist(
    const Eigen::Vector3d& point1
    , const Eigen::Vector3d& point2);

/**
 * @brief Fills the given array with a number of evenly spaced point coordinates from the given
 * start point using a step size (inspired by Python's numpy.linspace())
 * @note The given progress can shift the point coordinates to or from the start point
 * @note The array must have capacity to hold the number of point coordinates. Otherwise: boom! the programme
 * will crash.
 * @param array The array to fill point coordinates into
 * @param start_point The first point coordinate
 * @param num_packets The number of point coordinates needed
 * @param step The step size by which the next point coordinates are calculated from the start point
 * @param progress The amount to shift each point coordinate
 */
void linspace(
    double array[]
    , double start_point
    , int num_packets
    , double step
    , double progress);

/**
 * @brief Multiplies two 3 x 3 matrices
 * @param result The 2-D array to hold the result
 * @param matrix1 Operand 1
 * @param matrix2 Operand 2
 */
void multiplyMatrices(
    double result[3][3]
    , const double matrix1[3][3]
    , const double matrix2[3][3]);

/**
 * @brief Multiplies a 4 x 4 matrix by a 4 x 1 one
 * @param result The 2-D array to hold the result
 * @param matrix1 Operand 1
 * @param matrix2 Operand 2
 */
void multiplyMatrices(
    double result[4][1]
    , const double matrix1[4][4]
    , const double matrix2[4][1]);

/**
 * @brief Creates a 3-D (4 x 4) transform matrix based on the given translation and rotation
 * @param result A 2-D array to hold the resulting transform matrix
 * @param translation
 * @param rotation
 */
void getTransformMatrix(
    double result[4][4]
    , const double translation[3]
    , const double rotation[3]);

/**
 * @brief Transforms a 3-D point based on the given transform matrix
 * @param result An array to hold the resulting point
 * @param point
 * @param transform_matrix
 */
void transformPoint(
    double result[3]
    , const double point[3]
    , const double transform_matrix[4][4]);

/**
 * @brief Converts a ROS pose to arrays of xyz position and rpy orientation
 * @param pose
 * @param position
 * @param orientation
*/
void pose2Array(
    const geometry_msgs::msg::Pose& pose
    , double position[3]
    , double orientation[3]);

/**
 * @brief Convert geographic to MGRS
 * @param lon Longitude
 * @param lat Latitude
 * @param x MGRS x
 * @param y MGRS y
*/  
void toMgrs(double lon
, double lat
, double& x
, double& y);

} // namespace transforms

#endif // VISUALLY__TRANSFORMS_HPP_