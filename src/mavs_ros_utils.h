/**
* \file mavs_ros_uitls.h
*
* A set of functions for copying data from 
* MAVS data types to ROS data types
* and other utilities.
*
* \author Chris Goodin
*
* \date 9/1/2020
*/
#ifndef MAVS_ROS_UTILS_H
#define MAVS_ROS_UTILS_H
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
// mavs includes
#include <mavs_core/messages.h>
#include "CImg.h"
#ifdef Success
  #undef Success
#endif
namespace mavs_ros_utils{

/**
 * Get a heading (yaw) from a ROS quaternion.
 * \param orientation The normalized ROS quaternion 
 */
double GetHeadingFromOrientation(geometry_msgs::Quaternion orientation);

/**
 * Copy a MAVS PointCloud2 to a ROS PointCloud2 
 * \param pc The output ROS PointCloud2
 * \param mavs_pc The input MAVS PoinCloud2
 */
sensor_msgs::PointCloud2 CopyFromMavsPc2(mavs::PointCloud2 mavs_pc);

/**
 * Copy a MAVS Image to a ROS Image 
 * \param image The output ROS Image
 * \param mavs_image The input MAVS Image
 */
void CopyFromMavsImage(sensor_msgs::Image &image, mavs::Image &mavs_image);

/**
 * Copy a MAVS NavSatFix to a ROS NavSatFix 
 * \param fix The output ROS NavSatFix
 * \param mavs_fix The input MAVS NavSatFix
 */
void CopyFromMavsFix(sensor_msgs::NavSatFix &fix, mavs::NavSatFix &mavs_fix);

/**
 * Copy a MAVS Vehicle state to a ROS Odometry message
 * \param state The MAVS Vehicle State 
 */
nav_msgs::Odometry CopyFromMavsVehicleState(mavs::VehicleState state);

/**
 * Copy a MAVS Vehicle state to a ROS Odometry message
 * with velocities in body-fixed coordinates
 * \param state The MAVS Vehicle State
 */
nav_msgs::Odometry CopyFromMavsVehicleStateBodyFixed(mavs::VehicleState state);

/**
 * Copy a MAVS odometry message to a ROS Odometry message
 * \param odom The ROS output odometry message
 * \param mavs_odom The input MAVS odometry message
 */
void CopyFromMavsOdometry(nav_msgs::Odometry &odom, mavs::Odometry &mavs_odom);

/**
 * Copy a MAVS odometry message to a ROS Odometry message with body fixed velocities
 * \param odom The ROS output odometry message
 * \param mavs_odom The input MAVS odometry message
 */
void CopyFromMavsOdometryBodyFixed(nav_msgs::Odometry& odom, mavs::Odometry& mavs_odom);

/**
 * Copy a MAVS OccupancyGrid message to a ROS OccupancyGrid message
 * \param grid The ROS output OccupancyGrid message
 * \param mavs_grid The input MAVS OccupancyGrid message
 */
void CopyFromMavsGrid(nav_msgs::OccupancyGrid &grid, mavs::OccupancyGrid &mavs_grid);

/**
 * Calculate the distance from a point to a line.
 * \param x1 First point on the line
 * \param x2 Second point on the line
 * \param x0 Test point 
 */
double PointLineDistance(glm::dvec2 x1, glm::dvec2 x2, glm::dvec2 x0);

/**
 * Convert a CImg to a ROS Image message
 * \param in_image The input CImg image
 * \param out_image The output ROS sensor_msgs::Image data 
 */
template<typename T>
void CImgToImage(cimg_library::CImg<T> *in_image, sensor_msgs::Image &out_image){
	out_image.height = in_image->height();
	out_image.width = in_image->width();
	out_image.is_bigendian = false;
	out_image.encoding = "rgb8";
	if (out_image.height==0 || out_image.width==0){
		return;
	}
	out_image.data.resize(3 * out_image.height * out_image.width);
	int n = 0;
	for (int j =0; j <(int)out_image.height; j++){
		//for (int i=out_image.width-1; i>=0; i--){
		for (int i=0; i<out_image.width; i++){
			for (int k = 0; k < 3; k++){
				out_image.data[n] = (uint8_t)in_image->operator()(i,j,k);
				n++;
			}
		}
	}
	out_image.step = (uint32_t)(sizeof(uint8_t) * out_image.data.size() / out_image.height);
}

/**
 * Convert a ROS Image message to CImg
 * \param in_image The input ROS sensor_msgs::Image data
 * \param out_image The output CImg image 
 */
template<typename T>
void ImageToCImg(sensor_msgs::Image &in_image, cimg_library::CImg<T> &out_image){
	int channels = in_image.step/in_image.width;
	out_image.assign(in_image.width,in_image.height,1,channels,0);
	if (out_image.height()==0 || out_image.width()==0){
		return;
	}
	int n = 0;
	for (int j =0; j <(int)out_image.height(); j++){
		for (int i=0; i<out_image.width(); i++){
			for (int k = 0; k < channels; k++){
				out_image(i,j,k) = (T)in_image.data[n];
				n++;
			}
		}
	}
}

/**
 * Return distance from a point to a segment
 * \param ep1 First endpoint of the segment
 * \param ep2 Second endpoint of the segment
 * \param p The test point 
 */
double PointToSegmentDistance(glm::dvec2 ep1, glm::dvec2 ep2, glm::dvec2 p);

} //namespace mavs_ros_utils

#endif

