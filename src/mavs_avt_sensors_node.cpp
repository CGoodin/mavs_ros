// c++ includes
#include <omp.h>
#include <unistd.h>
//ros includes
#include "ros/ros.h"
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
	ros::init(argc, argv, "mavs_avt_sensors_node");
	ros::NodeHandle n;

	ros::Publisher clock_pub;

	ros::Subscriber odom_sub = n.subscribe("mavs_avt/odometry_true", 1, OdomCallback);
	ros::Publisher lidar_pub = n.advertise<sensor_msgs::PointCloud2>("mavs_avt/point_cloud2", 1);

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
	bool render_debug = false;
	if (ros::param::has("~debug_camera")){
		ros::param::get("~debug_camera", render_debug);
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
	std::string condition;
	if (ros::param::has("~condition")){
		ros::param::get("~condition", condition);
	}
	else{
		std::cerr << "ERROR: No condition name listed " << std::endl;
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

	mavs::vehicle::Rp3dVehicle mavs_veh;
	if (render_debug){
		mavs_veh.Load(rp3d_vehicle_file);
		mavs_veh.SetPosition(0.0, 0.0, 1.0);
		mavs_veh.SetOrientation(1.0, 0.0, 0.0, 0.0);
		mavs_veh.Update(&env, 0.0, 0.0, 1.0, 0.00001);
	}

	glm::vec3 offset(0.0f, 0.0f, 1.35f);
	glm::vec3 origin(0.0f, 0.0f, 0.f);
	glm::quat relor(1.0f, 0.0f, 0.0f, 0.0f);

/*
	mavs::sensor::camera::PathTracerCamera camera;
    camera.Initialize(1024, 576, 0.006222f, 0.0035f, 0.0035f);
    camera.SetNumIterations(1);
    camera.SetMaxDepth(10);
    camera.SetRRVal(0.55);
    camera.SetExposureTime(1.0f / 500.0f);
    camera.TurnOffPixelSmoothing();
*/
    mavs::sensor::camera::RgbCamera camera;
	camera.Initialize(512, 512, 0.0035, 0.0035, 0.0035);
	camera.SetRenderShadows(false);
	glm::vec3 cam_offset(-10.0, 0.0, 2.0);
	camera.SetRelativePose(cam_offset, relor);

	mavs::sensor::lidar::Lidar *lidar;

	if (lidar_type == "OS1"){
		lidar = new mavs::sensor::lidar::OusterOS1;
		lidar->SetName("OS1");
	}
	if (lidar_type == "OS2"){
		lidar = new mavs::sensor::lidar::OusterOS2;
		lidar->SetName("OS2");
	}
	if (lidar_type == "HDL32E"){
		lidar = new mavs::sensor::lidar::Hdl32E;
		lidar->SetName("HDL32E");
	}
	if (lidar_type == "HDL64E"){
		lidar = new mavs::sensor::lidar::Hdl64E;
		lidar->SetName("HDL64E");
	}
	if (lidar_type == "VLP16"){
		lidar = new mavs::sensor::lidar::Vlp16;
		lidar->SetName("VLP16");
	}
	if (lidar_type == "M8"){
		lidar = new mavs::sensor::lidar::MEight;
		lidar->SetName("M8");
	}
	else{
		lidar = new mavs::sensor::lidar::OusterOS1;
		lidar->SetName("OS1");
	}

	lidar->SetRelativePose(offset, relor);
	lidar->SetBlankingDist(blanking_dist);
	double dt = 0.1;
	ros::Rate rate(1.0 / dt);
	int nscans = 0;
	double elapsed_time = 0.0;
	double start_time = ros::Time::now().toSec();
	int frame_ctr = 0;
	std::ostringstream oss;
	while (ros::ok()){

		env.SetActorPosition(0, veh_state.pose.position, veh_state.pose.quaternion);
	
	
		if (render_debug){
			camera.SetPose(veh_state);
			camera.Update(&env, 0.033);
			camera.Display();
			oss.str("");
			oss << condition << "-image-";
			oss << std::setw(4) << std::setfill('0') << frame_ctr << ".bmp";
			std::cout << oss.str() << std::endl;
			camera.SaveImage("/scratch/dwc2/casestudy2021/" + oss.str());
			frame_ctr++;
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

		rate.sleep();
		ros::spinOnce();
	} //while ros OK

	double tot_time = ros::Time::now().toSec() - start_time;

	std::cout << "LIDAR rate = " << nscans / tot_time << " scans per second " << nscans << " " << tot_time << std::endl;

	delete lidar;

	return 0;
}
