#include "mavs_ros_utils.h"

#include <tf/transform_datatypes.h>
#include <iostream>

namespace mavs_ros_utils{

/**
 * Get a heading (yaw) from a ROS quaternion.
 * \param orientation The normalized ROS quaternion 
 */
double GetHeadingFromOrientation(geometry_msgs::Quaternion orientation){
    tf::Quaternion q(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
    return yaw;
}

/**
 * Copy a MAVS PointCloud2 to a ROS PointCloud2 
 * \param pc The output ROS PointCloud2
 * \param mavs_pc The input MAVS PoinCloud2
 */
sensor_msgs::PointCloud2 CopyFromMavsPc2(mavs::PointCloud2 mavs_pc){
    sensor_msgs::PointCloud2 pc;
	pc.height = mavs_pc.height;
	pc.width = mavs_pc.width;
	pc.is_bigendian = mavs_pc.is_bigendian;
	pc.point_step = mavs_pc.point_step;
	pc.row_step = mavs_pc.row_step;
	pc.is_dense = mavs_pc.is_dense;
	for (int i=0;i<mavs_pc.fields.size();i++){
		sensor_msgs::PointField field;
		field.name = mavs_pc.fields[i].name;
		field.offset = mavs_pc.fields[i].offset;
		field.datatype = mavs_pc.fields[i].datatype;
		field.count = mavs_pc.fields[i].count;
		pc.fields.push_back(field);
	}
	int data_size = pc.row_step*mavs_pc.height;
	pc.data.resize(data_size);
    //const unsigned char* bytes = reinterpret_cast<const unsigned char*>(&mavs_pc[0]);
	memcpy(&pc.data[0],&mavs_pc.data[0],data_size);
    //for (int i=0;i<pc.data.size(); i++){
    //    pc.data[i] = bytes[i]; //mavs_pc.data[i];
    //}
    return pc;
}

/**
 * Copy a MAVS Image to a ROS Image 
 * \param image The output ROS Image
 * \param mavs_image The input MAVS Image
 */
void CopyFromMavsImage(sensor_msgs::Image &image, mavs::Image &mavs_image){
    image.height = mavs_image.height;
    image.width = mavs_image.width;
    image.encoding = mavs_image.encoding;
    image.is_bigendian = mavs_image.is_bigendian;
    image.step = mavs_image.step;
    image.data = mavs_image.data;
}

/**
 * Copy a MAVS NavSatFix to a ROS NavSatFix 
 * \param fix The output ROS NavSatFix
 * \param mavs_fix The input MAVS NavSatFix
 */
void CopyFromMavsFix(sensor_msgs::NavSatFix &fix, mavs::NavSatFix &mavs_fix){
    fix.latitude = mavs_fix.latitude;
    fix.longitude = mavs_fix.longitude;
    fix.altitude = mavs_fix.altitude;
    fix.status.status = mavs_fix.status.status;
    fix.status.service = mavs_fix.status.service;
}

nav_msgs::Odometry CopyFromMavsVehicleState(mavs::VehicleState state){
	nav_msgs::Odometry odom;
	odom.pose.pose.position.x = state.pose.position.x;
	odom.pose.pose.position.y = state.pose.position.y;
	odom.pose.pose.position.z = state.pose.position.z;
	odom.pose.pose.orientation.x = state.pose.quaternion.x;
	odom.pose.pose.orientation.y = state.pose.quaternion.y;
	odom.pose.pose.orientation.z = state.pose.quaternion.z;
	odom.pose.pose.orientation.w = state.pose.quaternion.w;
	odom.twist.twist.linear.x = state.twist.linear.x;
	odom.twist.twist.linear.y = state.twist.linear.y;
	odom.twist.twist.linear.z = state.twist.linear.z;
	odom.twist.twist.angular.x = state.twist.angular.x;
	odom.twist.twist.angular.y = state.twist.angular.y;
	odom.twist.twist.angular.z = state.twist.angular.z;
	return odom;
}

nav_msgs::Odometry CopyFromMavsVehicleStateBodyFixed(mavs::VehicleState state) {
	nav_msgs::Odometry odom;
	odom.pose.pose.position.x = state.pose.position.x;
	odom.pose.pose.position.y = state.pose.position.y;
	odom.pose.pose.position.z = state.pose.position.z;
	odom.pose.pose.orientation.x = state.pose.quaternion.x;
	odom.pose.pose.orientation.y = state.pose.quaternion.y;
	odom.pose.pose.orientation.z = state.pose.quaternion.z;
	odom.pose.pose.orientation.w = state.pose.quaternion.w;

	double w = state.pose.quaternion.w;
	double x = state.pose.quaternion.x;
	double y = state.pose.quaternion.y; 
	double z = state.pose.quaternion.z;
	double R[3][3]; 
	R[0][0] = 1.0 - 2.0 * (y * y + z * z);
	R[0][1] = 2.0 * (x * y - z * w);
	R[0][2] = 2.0 * (x * z + y * w);

	R[1][0] = 2.0 * (x * y + z * w);
	R[1][1] = 1.0 - 2.0 * (x * x + z * z);
	R[1][2] = 2.0 * (y * z - x * w);

	R[2][0] = 2.0 * (x * z - y * w);
	R[2][1] = 2.0 * (y * z + x * w);
	R[2][2] = 1.0 - 2.0 * (x * x + y * y);

	double look_side[3] = { R[0][1], R[1][1], R[2][1] }; // East
	double look_to[3] = { R[0][0], R[1][0], R[2][0] }; // North
	double look_up[3] = { R[0][2], R[1][2], R[2][2] }; // Up
	double vx = state.twist.linear.x;
	double vy = state.twist.linear.y;
	double vz = state.twist.linear.z;

	odom.twist.twist.linear.x = vx * look_to[0] + vy * look_to[1] + vz * look_to[2];
	odom.twist.twist.linear.y = vx * look_side[0] + vy * look_side[1] + vz * look_side[2];
	odom.twist.twist.linear.z = vx * look_up[0] + vy * look_up[1] + vz * look_up[2];
	odom.twist.twist.angular.x = state.twist.angular.x;
	odom.twist.twist.angular.y = state.twist.angular.y;
	odom.twist.twist.angular.z = state.twist.angular.z;
	return odom;
}

void CopyFromMavsOdometry(nav_msgs::Odometry &odom, mavs::Odometry &mavs_odom){
	odom.pose.pose.position.x = mavs_odom.pose.pose.position.x;
	odom.pose.pose.position.y = mavs_odom.pose.pose.position.y;
	odom.pose.pose.position.z = mavs_odom.pose.pose.position.z;
	odom.pose.pose.orientation.w = mavs_odom.pose.pose.quaternion.w;
	odom.pose.pose.orientation.x = mavs_odom.pose.pose.quaternion.x;
	odom.pose.pose.orientation.y = mavs_odom.pose.pose.quaternion.y;
	odom.pose.pose.orientation.z = mavs_odom.pose.pose.quaternion.z;
	odom.twist.twist.linear.x = mavs_odom.twist.twist.linear.x;
	odom.twist.twist.linear.y = mavs_odom.twist.twist.linear.y;
	odom.twist.twist.linear.z = mavs_odom.twist.twist.linear.z;
	odom.twist.twist.angular.x = mavs_odom.twist.twist.angular.x;
	odom.twist.twist.angular.y = mavs_odom.twist.twist.angular.y;
	odom.twist.twist.angular.z = mavs_odom.twist.twist.angular.z;
}

void CopyFromMavsOdometryBodyFixed(nav_msgs::Odometry& odom, mavs::Odometry& mavs_odom) {
	odom.pose.pose.position.x = mavs_odom.pose.pose.position.x;
	odom.pose.pose.position.y = mavs_odom.pose.pose.position.y;
	odom.pose.pose.position.z = mavs_odom.pose.pose.position.z;
	odom.pose.pose.orientation.w = mavs_odom.pose.pose.quaternion.w;
	odom.pose.pose.orientation.x = mavs_odom.pose.pose.quaternion.x;
	odom.pose.pose.orientation.y = mavs_odom.pose.pose.quaternion.y;
	odom.pose.pose.orientation.z = mavs_odom.pose.pose.quaternion.z;

	double w = mavs_odom.pose.pose.quaternion.w;
	double x = mavs_odom.pose.pose.quaternion.x;
	double y = mavs_odom.pose.pose.quaternion.y;
	double z = mavs_odom.pose.pose.quaternion.z;
	double R[3][3];
	R[0][0] = 1.0 - 2.0 * (y * y + z * z);
	R[0][1] = 2.0 * (x * y - z * w);
	R[0][2] = 2.0 * (x * z + y * w);

	R[1][0] = 2.0 * (x * y + z * w);
	R[1][1] = 1.0 - 2.0 * (x * x + z * z);
	R[1][2] = 2.0 * (y * z - x * w);

	R[2][0] = 2.0 * (x * z - y * w);
	R[2][1] = 2.0 * (y * z + x * w);
	R[2][2] = 1.0 - 2.0 * (x * x + y * y);

	double look_side[3] = { R[0][1], R[1][1], R[2][1] }; // East
	double look_to[3] = { R[0][0], R[1][0], R[2][0] }; // North
	double look_up[3] = { R[0][2], R[1][2], R[2][2] }; // Up
	double vx = mavs_odom.twist.twist.linear.x;
	double vy = mavs_odom.twist.twist.linear.y;
	double vz = mavs_odom.twist.twist.linear.z;

	odom.twist.twist.linear.x = vx * look_to[0] + vy * look_to[1] + vz * look_to[2];
	odom.twist.twist.linear.y = vx * look_side[0] + vy * look_side[1] + vz * look_side[2];
	odom.twist.twist.linear.z = vx * look_up[0] + vy * look_up[1] + vz * look_up[2];
	
	odom.twist.twist.angular.x = mavs_odom.twist.twist.angular.x;
	odom.twist.twist.angular.y = mavs_odom.twist.twist.angular.y;
	odom.twist.twist.angular.z = mavs_odom.twist.twist.angular.z;
}

void CopyFromMavsGrid(nav_msgs::OccupancyGrid &grid, mavs::OccupancyGrid &mavs_grid){

    grid.info.width = mavs_grid.info.width;
    grid.info.height = mavs_grid.info.height;
    grid.info.resolution = mavs_grid.info.resolution;
    grid.info.origin.position.x = mavs_grid.info.origin.position.x; 
    grid.info.origin.position.y = mavs_grid.info.origin.position.y;
    grid.info.origin.position.z = mavs_grid.info.origin.position.z;
    grid.info.origin.orientation.w = mavs_grid.info.origin.quaternion.w;
    grid.info.origin.orientation.x = mavs_grid.info.origin.quaternion.x;
    grid.info.origin.orientation.y = mavs_grid.info.origin.quaternion.y;
    grid.info.origin.orientation.z = mavs_grid.info.origin.quaternion.z;

	grid.data.resize(mavs_grid.data.size());
	for (int i=0;i<grid.info.width;i++){
		for (int j=0;j<grid.info.height;j++){
			int n0 = grid.info.width*j + i;
			int n1 = grid.info.height*i + j;
			grid.data[n0] = mavs_grid.data[n1];
		}
	}

    //grid.data.resize(mavs_grid.data.size());
    //for (int i=0;i<mavs_grid.data.size();i++){
    //    grid.data[i] = (uint8_t)mavs_grid.data[i];
    //}
    grid.header.frame_id = "map";
}


double PointLineDistance(glm::dvec2 x1, glm::dvec2 x2, glm::dvec2 x0) {
	glm::dvec3 x01(x0.x - x1.x, x0.y - x1.y, 0.0);
	glm::dvec3 x02(x0.x - x2.x, x0.y - x2.y, 0.0);
	glm::dvec2 x21 = x2 - x1;
	double d = glm::length(glm::cross(x01, x02)) / glm::length(x21);
	return d;
}

/**
 * Return distance from a point to a segment
 * \param ep1 First endpoint of the segment
 * \param ep2 Second endpoint of the segment
 * \param p The test point 
 */
double PointToSegmentDistance(glm::dvec2 ep1, glm::dvec2 ep2, glm::dvec2 p) {
	glm::dvec2 v21 = ep2 - ep1;
	glm::dvec2 pv1 = p - ep1;
	if (glm::dot(v21, pv1) <= 0.0) {
		double d = glm::length(pv1);
		return d;
	}
	glm::dvec2 v12 = ep1 - ep2;
	glm::dvec2 pv2 = p - ep2;
	if (glm::dot(v12, pv2) <= 0.0) {
		double d = glm::length(pv2);
		return d;
	}
	double d0 = PointLineDistance(ep1, ep2, p);
	return d0;
}

} //namespace mavs_ros_utils


