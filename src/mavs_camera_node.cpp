// c++ includes
#include <omp.h>
#include <unistd.h>
//ros includes
#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include <std_msgs/Int64.h>
#include "stereo_msgs/DisparityImage.h"
#include "sensor_msgs/CameraInfo.h"
#include <std_msgs/Int64.h>
#include <tf/transform_broadcaster.h>
// local includes
#include "mavs_ros_utils.h"
// mavs includes
#include "sensors/sensor.h"
#include "sensors/mavs_sensors.h"
#include "sensors/camera/path_tracer.h"
#include "raytracers/embree_tracer/embree_tracer.h"
#include "mavs_core/environment/particle_system/particle_system.h"
#include "mavs_core/math/utils.h"
#include "sensors/lidar/lidar_tools.h"
#include "vehicles/rp3d_veh/mavs_rp3d_veh.h"
#include "mavs_core/data_path.h"

float human_throttle = 0.0;
float human_steering = 0.0;
float human_braking = 0.0;

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

std::map<std::string, geometry_msgs::PoseArray > poses_map;
void AnimPosesCallback(const geometry_msgs::PoseArray::ConstPtr& rcv_msg){
	geometry_msgs::PoseArray anim_poses = *rcv_msg;
	std::string key = anim_poses.header.frame_id;
	if (poses_map.count(key)>0 ){
		poses_map[key] = anim_poses;
	}
	else{
		poses_map.insert({key,anim_poses});	
	}
}

int main(int argc, char **argv){
	//- Create the node and subscribers ---//
	ros::init(argc, argv, "mavs_camera_node");
	ros::NodeHandle n;

	ros::Publisher clock_pub;

	ros::Subscriber odom_sub = n.subscribe("mavs_ros/odometry_true", 1, OdomCallback);
	ros::Publisher camera_pub = n.advertise<sensor_msgs::Image>("mavs_ros/image", 1);
	ros::Publisher dc_pub = n.advertise<geometry_msgs::Twist>("mavs_ros/cmd_vel", 1);


	int num_veh = 1;
	if (ros::param::has("~num_vehicles")){
		ros::param::get("~num_vehicles",num_veh);
	}

	std::vector<ros::Subscriber> anim_subs;
	for (int nv = 0; nv<num_veh;nv++){
		std::string topic_name = "/mavs_ros/anim_poses"+mavs::utils::ToString(nv+1,3);
		auto anim_sub = n.subscribe(topic_name, num_veh+25, AnimPosesCallback);
		anim_subs.push_back(anim_sub);
	}

	std::string scene_file;
	if (ros::param::has("~scene_file")){
		ros::param::get("~scene_file", scene_file);
	}
	else{
		std::cerr << "ERROR: No scene file listed " << std::endl;
	}
	float rain_rate = 0.0f;
	if (ros::param::has("~rain_rate")){
		ros::param::get("~rain_rate", rain_rate);
	}
	float dust_rate = 0.0f;
	if (ros::param::has("~dust_rate")){
		ros::param::get("~dust_rate", dust_rate);
	}

	bool display_image = false;
	if (ros::param::has("~display_image")){
		ros::param::get("~display_image", display_image);
	}

	bool publish_driving_commands = false;
	if (ros::param::has("~publish_driving_commands")){
		ros::param::get("~publish_driving_commands", publish_driving_commands);
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

	int num_horizontal_pix = 256;
	if (ros::param::has("~num_horizontal_pix")){
		ros::param::get("~num_horizontal_pix",num_horizontal_pix);
	}
	int num_vertical_pix = 256;
	if (ros::param::has("~num_vertical_pix")){
		ros::param::get("~num_vertical_pix",num_vertical_pix);
	}
	float horizontal_pixdim = 0.0035f;
	if (ros::param::has("~horizontal_pixdim")){
		ros::param::get("~horizontal_pixdim",horizontal_pixdim);
	}
	float vertical_pixdim = 0.0035f;
	if (ros::param::has("~vertical_pixdim")){
		ros::param::get("~vertical_pixdim",vertical_pixdim);
	}
	float focal_length = 0.0035f;
	if (ros::param::has("~focal_length")){
		ros::param::get("~focal_length",focal_length);
	}
	std::string sensor_name="camera";
	if (ros::param::has("~sensor_name")){
		ros::param::get("~sensor_name", sensor_name);
	}
	std::string rp3d_vehicle_file;
	if (ros::param::has("~rp3d_vehicle_file")){
		ros::param::get("~rp3d_vehicle_file", rp3d_vehicle_file);
	}
	else{
		std::cerr << "ERROR: MUST PROVIDE PARAMETER rp3d_vehicle_file to use mavs node" << std::endl;
		exit(1);
	}
	

	mavs::environment::Environment env;
	if (rain_rate > 0.0){
		env.SetRainRate(rain_rate);
		float wx = mavs::math::rand_in_range(-3.0f, 3.0f);
		float wy = mavs::math::rand_in_range(-3.0f, 3.0f);
		env.SetWind(wx, wy);
	}
	if (dust_rate > 0.0){
		mavs::environment::ParticleSystem dust;
		dust.Dust();
		dust.SetSource(glm::vec3(0.0, 0.0, 1.25), 1.0, dust_rate);
		dust.SetInitialVelocity(-3.0, 0.0, 2.5);
		dust.SetVelocityRandomization(0.25, 0.25, 0.25);
		env.AddParticleSystem(dust);
	}
	mavs::raytracer::embree::EmbreeTracer scene;
	scene.Load(scene_file); 
	scene.TurnOffLabeling();
	env.SetRaytracer(&scene);

	for (int nv =0; nv<(int)num_veh;nv++){
		mavs::vehicle::Rp3dVehicle mavs_veh;
		mavs_veh.Load(rp3d_vehicle_file);
		mavs_veh.SetPosition(-10000.0f, -10000.0f, -10000.0f);
		mavs_veh.SetOrientation(1.0f, 0.0f, 0.0f, 0.0f);
		mavs_veh.Update(&env, 0.0, 0.0, 0.0, 0.00001);
	}

	glm::vec3 offset(off_x, off_y, off_z);
	glm::quat relor(1.0f, 0.0f, 0.0f, 0.0f);

	mavs::sensor::camera::RgbCamera camera;
	camera.Initialize(num_horizontal_pix,num_vertical_pix, horizontal_pixdim, vertical_pixdim, focal_length);
	camera.SetName(sensor_name);
	camera.SetRelativePose(offset, relor);

	double dt = 0.1;
	ros::Rate rate(1.0 / dt);
	int nscans = 0;
	double elapsed_time = 0.0;
	double start_time = ros::Time::now().toSec();
	while (ros::ok()){

		if ((int)poses_map.size()==num_veh || num_veh==1){
			if (num_veh==1){
				env.SetActorPosition(0, veh_state.pose.position, veh_state.pose.quaternion);
			}
			else{
				// consolidate all the animation poses
				geometry_msgs::PoseArray anim_poses;
				//anim_poses.header.stamp = n->now();
				anim_poses.header.frame_id = "world";
				for (auto const& x : poses_map){
					for (int i=0;i<(int)x.second.poses.size();i++){
						anim_poses.poses.push_back(x.second.poses[i]);
					}
				}
				// move all the actors
				if (env.GetNumberOfActors()>=(int)(anim_poses.poses.size())){
					for (int i=0;i<(int)anim_poses.poses.size();i++){
						glm::vec3 tpos(anim_poses.poses[i].position.x, anim_poses.poses[i].position.y, anim_poses.poses[i].position.z);
						glm::quat tori(anim_poses.poses[i].orientation.w, anim_poses.poses[i].orientation.x, anim_poses.poses[i].orientation.y, anim_poses.poses[i].orientation.z);
						env.SetActorPosition(i, tpos, tori, dt, true);
					}
				}
			}

			double t0 = omp_get_wtime();
			env.AdvanceParticleSystems(0.1);

			camera.SetPose(veh_state);
			
			camera.Update(&env, 0.1);
		
			mavs::Image mavs_img;
			
			sensor_msgs::Image img;
			mavs_ros_utils::CopyFromMavsImage(img, mavs_img);



			if (publish_driving_commands){	
				std::vector<bool> driving_commands = camera.GetKeyCommands();
				if (driving_commands[0]) {
					//human_throttle = 1.0f; 
					human_throttle += dt/2.0f;
					human_throttle = std::min(1.0f, human_throttle);
					human_braking = 0.0f;
				}
				else if (driving_commands[1]) { 
					human_braking -= dt;
					human_braking = std::max(-1.0f, human_braking);
					human_throttle = 0.0f;
				}
				else{ 
					human_throttle *= 0.5f;
					human_braking = 0.0f;
				}
				if (driving_commands[2]) {
					human_steering += dt;
					human_steering = std::min(1.0f, human_steering);
				}
				else if (driving_commands[3]) { 
					human_steering -= dt;
					human_steering = std::max(-1.0f, human_steering);
				}
				else{
					human_steering = 0.0f; 
				}
				geometry_msgs::Twist dc;
				dc.linear.x = human_throttle;
				dc.linear.y = human_braking;
				dc.angular.z = human_steering;
				dc_pub.publish(dc);
			}

			nscans++;

			img.header.stamp = ros::Time::now();
			img.header.frame_id = "odom";
			camera_pub.publish(img);

			if (display_image){
				camera.Display();
			}
		}
		rate.sleep();
		ros::spinOnce();
	} //while ros OK

	double tot_time = ros::Time::now().toSec() - start_time;

	std::cout << "Camera rate = " << nscans / tot_time << " images per second " << nscans << " " << tot_time << std::endl;

	return 0;
}
