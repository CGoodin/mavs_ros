//ros includes
#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
// mavs includes
#include "sensors/gps/gps.h"
#include "raytracers/embree_tracer/embree_tracer.h"

mavs::VehicleState veh_state; 
void OdomCallback(const nav_msgs::Odometry::ConstPtr& rcv_odom){
	nav_msgs::Odometry odom = *rcv_odom;
	veh_state.pose.position.x = rcv_odom->pose.pose.position.x;
	veh_state.pose.position.y = rcv_odom->pose.pose.position.y;
	veh_state.pose.position.z = rcv_odom->pose.pose.position.z;
	veh_state.pose.quaternion.w = rcv_odom->pose.pose.orientation.w;
	veh_state.pose.quaternion.x = rcv_odom->pose.pose.orientation.x;
	veh_state.pose.quaternion.y = rcv_odom->pose.pose.orientation.y;
	veh_state.pose.quaternion.z = rcv_odom->pose.pose.orientation.z;
}

int main(int argc, char **argv){
	//- Create the node and subscribers ---//
	ros::init(argc, argv, "mavs_gps_node");
	ros::NodeHandle n;

	ros::Subscriber odom_sub = n.subscribe("mavs_ros/odometry_true", 1, OdomCallback);
	ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>("mavs_ros/navsatfix", 1);

	std::string scene_file;
	if (ros::param::has("~scene_file")){
		ros::param::get("~scene_file", scene_file);
	}
	else{
		std::cerr << "ERROR: No scene file listed " << std::endl;
	}

	bool display_image = false;
	if (ros::param::has("~display_image")){
		ros::param::get("~display_image", display_image);
	}

	float off_x = 1.0f;
	if (ros::param::has("~off_x")){
		ros::param::get("~off_x",off_x);
	}
	float off_y = 0.0f;
	if (ros::param::has("~off_y")){
		ros::param::get("~off_y",off_y);
	}

	float off_z = 1.5f;
	if (ros::param::has("~off_z")){
		ros::param::get("~off_z",off_z);
	}

	double local_origin_latitude = 32.3033;
	double local_origin_longitude = 90.8742;
	double local_origin_altitude = 73.152;
	if (ros::param::has("~local_origin_latitude")) ros::param::get("~local_origin_latitude", local_origin_latitude);
	if (ros::param::has("~local_origin_longitude")) ros::param::get("~local_origin_longitude", local_origin_latitude);
	if (ros::param::has("~local_origin_altitude")) ros::param::get("~local_origin_altitude", local_origin_latitude);

	std::string gps_mode = "dual band"; // or "differential
	if (ros::param::has("~gps_mode")) {
		ros::param::get("~gps_mode", gps_mode);
	}

	mavs::raytracer::embree::EmbreeTracer scene;
	scene.Load(scene_file); 
	scene.TurnOffLabeling();

	mavs::environment::Environment env;
	env.SetRaytracer(&scene);
	env.SetLocalOrigin(local_origin_latitude, local_origin_longitude, local_origin_altitude);

	glm::vec3 offset(off_x, off_y, off_z);
	glm::quat relor(1.0f, 0.0f, 0.0f, 0.0f);

	mavs::sensor::gps::Gps gps;
	gps.SetType(gps_mode);

	double dt = 1.0;
	ros::Rate rate(1.0 / dt);

	while (ros::ok()){

		env.AdvanceTime(dt);

		gps.SetPose(veh_state);
			
		gps.Update(&env, dt);

		glm::dvec3 lla = gps.GetRecieverPositionLLA();
		
		mavs::NavSatStatus nav_status = gps.GetRosNavSatStatus();

		sensor_msgs::NavSatFix fix;
		fix.latitude = lla.x;
		fix.longitude = lla.y;
		fix.altitude = lla.z;
		fix.header.frame_id = "world";
		fix.status.status = nav_status.status;
		fix.status.service = nav_status.service;
		gps_pub.publish(fix);

		if (display_image){
			gps.Display();
		}
		
		rate.sleep();
		ros::spinOnce();
	} //while ros OK

	return 0;
}
