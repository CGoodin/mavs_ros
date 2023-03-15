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
	ros::init(argc, argv, "mavs_sensors_node");
	ros::NodeHandle n;

	ros::Publisher clock_pub;

	ros::Subscriber odom_sub = n.subscribe("mavs_ros/odometry_true", 1, OdomCallback);
	ros::Publisher lidar_pub = n.advertise<sensor_msgs::PointCloud2>("mavs_ros/point_cloud2", 1);


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
	bool use_lidar = true;
	if (ros::param::has("~use_lidar")){
		ros::param::get("~use_lidar", use_lidar);
	}
	bool display_lidar = false;
	if (ros::param::has("~display_lidar")){
		ros::param::get("~display_lidar", display_lidar);
	}
	bool register_lidar = true;
	if (ros::param::has("~register_lidar")){
		ros::param::get("~register_lidar", register_lidar);
	}
	std::string lidar_type = "OS1";
	if (ros::param::has("~lidar_type")){
		ros::param::get("~lidar_type", lidar_type);
	}
	float blanking_dist = 1.0f;
	if (ros::param::has("~blanking_dist")){
		ros::param::get("~blanking_dist",blanking_dist);
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

	float grid_size = 500.0f;
	if (ros::param::has("~grid_size")){
		ros::param::get("~grid_size", grid_size);
	}
	float grid_res = 1.0f;
	if (ros::param::has("~grid_res")){
		ros::param::get("~grid_res", grid_res);
	}
	std::string rp3d_vehicle_file;
	if (ros::param::has("~rp3d_vehicle_file")){
		ros::param::get("~rp3d_vehicle_file", rp3d_vehicle_file);
	}
	else{
		std::cerr << "ERROR: MUST PROVIDE PARAMETER rp3d_vehicle_file to use mavs node" << std::endl;
		exit(1);
	}
	std::string sensor_name="lidar";
	if (ros::param::has("~sensor_name")){
		ros::param::get("~sensor_name", sensor_name);
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


	//mavs::MavsDataPath mdp;
	//std::string mavs_data_path = mdp.GetPath();
	for (int nv =0; nv<(int)num_veh;nv++){
		mavs::vehicle::Rp3dVehicle mavs_veh;
		mavs_veh.Load(rp3d_vehicle_file);
		mavs_veh.SetPosition(-10000.0f, -10000.0f, -10000.0f);
		mavs_veh.SetOrientation(1.0f, 0.0f, 0.0f, 0.0f);
		mavs_veh.Update(&env, 0.0, 0.0, 0.0, 0.00001);
	}

	glm::vec3 offset(off_x, off_y, off_z);
	glm::quat relor(1.0f, 0.0f, 0.0f, 0.0f);

	mavs::sensor::lidar::Lidar *lidar;

	if (lidar_type == "OS1"){
		lidar = new mavs::sensor::lidar::OusterOS1;
		//lidar->SetName("OS1");
	}
	if (lidar_type == "OS2"){
		lidar = new mavs::sensor::lidar::OusterOS2;
		//lidar->SetName("OS2");
	}
	if (lidar_type == "HDL32E"){
		lidar = new mavs::sensor::lidar::Hdl32E;
		//lidar->SetName("HDL32E");
	}
	if (lidar_type == "HDL64E"){
		lidar = new mavs::sensor::lidar::Hdl64E;
		//lidar->SetName("HDL64E");
	}
	if (lidar_type == "VLP16"){
		lidar = new mavs::sensor::lidar::Vlp16;
		//lidar->SetName("VLP16");
	}
	if (lidar_type == "M8"){
		lidar = new mavs::sensor::lidar::MEight;
		//lidar->SetName("M8");
	}
	else{
		lidar = new mavs::sensor::lidar::OusterOS1;
		//lidar->SetName("OS1");
	}
	lidar->SetName(sensor_name);
	lidar->SetRelativePose(offset, relor);
	lidar->SetBlankingDist(blanking_dist);
	double dt = 0.1;
	ros::Rate rate(1.0 / dt);
	int nscans = 0;
	double elapsed_time = 0.0;
	double start_time = ros::Time::now().toSec();
	while (ros::ok()){

		//env.SetActorPosition(0, veh_state.pose.position, veh_state.pose.quaternion);

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

			lidar->SetPose(veh_state);
			
			lidar->Update(&env, 0.1);
		
			mavs::PointCloud2 mavs_pc;
			if (register_lidar){
				mavs_pc = lidar->GetPointCloud2Registered(); 
			}
			else{
				mavs_pc = lidar->GetPointCloud2();
			}
			sensor_msgs::PointCloud2 pc = mavs_ros_utils::CopyFromMavsPc2(mavs_pc);

			nscans++;

			pc.header.stamp = ros::Time::now();
			pc.header.frame_id = "odom";
			lidar_pub.publish(pc);

			if (display_lidar){
				lidar->Display();
			}
		}
		rate.sleep();
		ros::spinOnce();
	} //while ros OK

	double tot_time = ros::Time::now().toSec() - start_time;

	std::cout << "LIDAR rate = " << nscans / tot_time << " scans per second " << nscans << " " << tot_time << std::endl;

	delete lidar;

	return 0;
}
