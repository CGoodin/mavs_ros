// c++ includes
#include <omp.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
//ros includes
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "rosgraph_msgs/Clock.h"
#include <std_msgs/Int64.h>
#include <tf/transform_broadcaster.h>
// local includes
#include "mavs_ros_utils.h"
// mavs includes
#include "mavs_core/math/utils.h"
#include "vehicles/rp3d_veh/mavs_rp3d_veh.h"
#include "raytracers/embree_tracer/embree_tracer.h"
#include "sensors/mavs_sensors.h"

float throttle = 0.0f;
float steering = 0.0f;
float braking = 0.0f;
void TwistCallback(const geometry_msgs::Twist::ConstPtr &rcv_msg){
	throttle = rcv_msg->linear.x;
	braking = rcv_msg->linear.y;
	steering = rcv_msg->angular.z;
}

bool CheckRollover(mavs::vehicle::Rp3dVehicle *veh) {
		glm::vec3 look_up = veh->GetLookUp();
		if (look_up.z < 0.0f) {
			std::cout << "VEHICLE ROLLED OVER" << std::endl;
			// Next line will end the simulation
			return true;
		}
		return false;
	}

bool CheckCollisions(mavs::vehicle::Rp3dVehicle *veh, mavs::environment::Environment *env) {
	glm::vec3 veh_pos = veh->GetPosition();
	// The offset of 2 is to account for the surface meshes, which don't line up with the bounding box list
	for (int i = 0; i < env->GetNumberObjects()-2; i++) {
		std::string label = env->GetLabel(env->GetObjectName(i));
		// exclude some labels that we don't want to treat as obstacles
		if (label != "" && label != "smooth trail" && label != "rough trail" && label != "vehicle" && label != "surface" && label != "pothole") {
			mavs::raytracer::BoundingBox box = env->GetObjectBoundingBox(i+2);
			glm::vec3 ll = box.GetLowerLeft();
			glm::vec3 ur = box.GetUpperRight();
			if (veh_pos.x > ll.x && veh_pos.x<ur.x && veh_pos.y>ll.y && veh_pos.y < ur.y) {
				std::cout << "COLLISION WITH OBJECT " << i << ", Obj: "<< env->GetObjectName(i) << " ";
				std::cout << ll.x << "," << ll.y << " " << ur.x << ", " << ur.y << " Veh: " << veh_pos.x << ", " << veh_pos.y << std::endl;
				// Next line will end the simulation
				return true;
			} 
		}
	}
	return false;
}

inline float GetHeadingFromOrientation(glm::quat orientation){
    tf::Quaternion q(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
    return (float)yaw;
}

int main(int argc, char **argv){
	//- Create the node and subscribers ---//
	ros::init(argc, argv, "mavs_avt_vehicle_node");
	ros::NodeHandle n;
	
	ros::Publisher clock_pub;
	ros::Subscriber twist_sub = n.subscribe("mavs_avt/cmd_vel", 1, TwistCallback);

	ros::Publisher odom_true = n.advertise<nav_msgs::Odometry>("mavs_avt/odometry_true", 10);
	ros::Publisher rtk_pub;

	rtk_pub = n.advertise<nav_msgs::Odometry>("mavs_avt/odometry", 1);

	//--- get parameters ---//
	bool use_sim_time = false;
	if (n.hasParam("use_sim_time")){
		n.getParam("use_sim_time", use_sim_time);
	}
	if (use_sim_time){
		clock_pub = n.advertise<rosgraph_msgs::Clock>("clock", 1);
	}

	int num_rays = 10;
	if (ros::param::has("~num_rays")) {
		ros::param::get("~num_rays", num_rays);
	} else {
		std::cerr << "ERROR: Number of rays not listed" << std::endl;
	}
	std::string scene_file;
	if (ros::param::has("~scene_file")){
		ros::param::get("~scene_file", scene_file);
	}
	else{
		std::cerr << "ERROR: No scene file listed " << std::endl;
	}

	std::string condition;
	if (ros::param::has("~condition")){
		ros::param::get("~condition", condition);
	}
	else{
		std::cerr << "ERROR: No trial name listed " << std::endl;
	}

	std::string trial_name;
	if (ros::param::has("~trial_name")){
		ros::param::get("~trial_name", trial_name);
	}
	else{
		std::cerr << "ERROR: No trial name listed " << std::endl;
	}

	float soil_strength = 250.0f;
	if (ros::param::has("~soil_strength")){
		ros::param::get("~soil_strength", soil_strength);
	}
	std::string surface_type = "dry";
	if (ros::param::has("~surface_type")){
		ros::param::get("~surface_type", surface_type);
	}
	float rain_rate = 0.0f;
	if (ros::param::has("/mavs_avt_sensors_node/rain_rate")){
		ros::param::get("/mavs_avt_sensors_node/rain_rate", rain_rate);
	} else {
		std::cerr << "ERROR: No Rain Rate Found" << std::endl;
	}

	float x_init = -45.0f;
	if (ros::param::has("~Initial_X_Position")){
		ros::param::get("~Initial_X_Position", x_init);
	}
	float y_init = 0.0f;
	if (ros::param::has("~Initial_Y_Position")){
		ros::param::get("~Initial_Y_Position", y_init);
	}
	float heading_init = 0.0f;
	if (ros::param::has("~Initial_Heading")){
		ros::param::get("~Initial_Heading", heading_init);
	}

	bool render_debug = false;
	if (ros::param::has("~debug_camera")){
		ros::param::get("~debug_camera", render_debug);
	}

	glm::vec3 initial_position(x_init, y_init, 1.0f);
	glm::quat initial_orientation(cos(0.5 * heading_init), 0.0f, 0.0f, sin(0.5 * heading_init));
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

	mavs::raytracer::embree::EmbreeTracer scene;
	scene.Load(scene_file); //, rand_seed);
	scene.TurnOffLabeling();

	env.SetRaytracer(&scene);
	env.SetGlobalSurfaceProperties(surface_type, 6894.76f * soil_strength);

	glm::vec3 origin(0.0f, 0.0f, 0.f);
	glm::quat relor(1.0f, 0.0f, 0.0f, 0.0f);

	mavs::sensor::rtk::Rtk piksi;
	float dropout_rate = 0.0f;
	float rtk_max_error = 1.0f;
	piksi.SetDropoutRate(dropout_rate);
	piksi.SetError(rtk_max_error);
	piksi.SetWarmupTime(660.0f);
	piksi.SetRelativePose(origin, relor);

	mavs::vehicle::Rp3dVehicle mavs_veh;
	mavs_veh.Load(rp3d_vehicle_file);
	mavs_veh.SetPosition(initial_position.x, initial_position.y, initial_position.z);
	mavs_veh.SetOrientation(initial_orientation.w, initial_orientation.x, initial_orientation.y, initial_orientation.z);
	mavs_veh.Update(&env, throttle, steering, braking, 0.00001);

	mavs::sensor::camera::RgbCamera camera;
	camera.Initialize(128, 128, 0.0035, 0.0035, 0.0035);
	camera.SetRenderShadows(false);
	glm::vec3 cam_offset(-10.0, 0.0, 2.0);
	camera.SetRelativePose(cam_offset, relor);

	double dt = 0.01;
	ros::Rate rate(1.0 / dt);
	int nsteps = 0;
	double elapsed_time = 0.0;

	// Logging
	std::ofstream experimentLogFile;
	experimentLogFile.open("/cavs/projects/ARC/Project1.31/users/dwc2/2021casestudy/data/results/" + condition + "_" + trial_name + "-experiment.csv", std::ios::out | std::ios::trunc);
	experimentLogFile << "Time,PosX,PosY,Heading,Speed,Distance,Throttle,Steering,Braking\n";
  	//experimentLogFileName = "../results/" + waypointFile.split('.')[0] + "-experiment.csv"
  	double distance_traveled = 0.0f;
	float prev_x = x_init;
	float prev_y = y_init;

	double start_time = ros::Time::now().toSec();
	uint64_t start_time_nSec = ros::Time::now().toNSec();
	std::string outcome = "SUCCESS";
	float sum_throttle = 0.0f;
	bool end_state = false;
/*
	mavs::sensor::camera::PathTracerCamera camera;
    camera.Initialize(1024, 576, 0.006222f, 0.0035f, 0.0035f);
    camera.SetNumIterations(num_rays);
    camera.SetMaxDepth(10);
    camera.SetRRVal(0.55);
    camera.SetExposureTime(1.0f / 500.0f);
    camera.TurnOffPixelSmoothing();
	glm::vec3 cam_offset(-10.0, 0.0, 2.0);
	camera.SetRelativePose(cam_offset, relor);
*/
	int loop_ctr = 0;
	int frame_ctr = 0;
	std::ostringstream oss;
	while (ros::ok() && !end_state){
		//vehicle state update
		mavs_veh.Update(&env, throttle, steering, -braking, dt);
		mavs::VehicleState veh_state = mavs_veh.GetState();

/*
		loop_ctr = loop_ctr + 1;
		if (loop_ctr % 10 == 0) {
			camera.SetPose(veh_state);
			camera.Update(&env, 0.033);
			oss.str("");
			oss << std::setw(4) << std::setfill('0') << frame_ctr;
			std::cout << "Filename: " << trial_name << "-" << oss.str() << ".bmp" << std::endl;
			camera.SaveImage("/scratch/dwc2/casestudy2021/" + condition + "-" + trial_name + "-" + oss.str() + ".bmp");
			frame_ctr++;
			camera.Display();
			loop_ctr = 0;
		}
*/
		// save state data: Time,PosX,PosY,Heading-Rad,Speed,DistanceTraveled
		float veh_x_ = veh_state.pose.position.x;
		float veh_y_ = veh_state.pose.position.y;
		float vx = veh_state.twist.linear.x;
		float vy = veh_state.twist.linear.y;
		float veh_speed_ = sqrt(vx*vx + vy*vy);
		float veh_heading_ = GetHeadingFromOrientation(veh_state.pose.quaternion);
		float dtx = prev_x - veh_x_;
		float dty = prev_y - veh_y_;
		prev_x = veh_x_;
		prev_y = veh_y_;
		double dtravel = sqrt(dtx*dtx + dty*dty);
		distance_traveled = distance_traveled + dtravel;

		if(veh_x_ < 0.0f || veh_x_ > 1000.0f || veh_y_ < 0.0f || veh_y_ > 1000.0f) {
			outcome = "OFFMAP";
			end_state = true;
		} 
		/*
		else if(veh_speed_ > 0.0f) {
			if(CheckRollover(&mavs_veh)) {
				outcome = "ROLLOVER";
				end_state = true;
			}
			if(CheckCollisions(&mavs_veh, &env)) {
				outcome = "COLLISION";
				end_state = true;
			}
		} else if (veh_speed_ < 0.1f ){
			outcome = "NEGATIVE SPEED";
			end_state = true;
		}
		*/
		
		if (ros::Time::now().toSec() - start_time > 600) {
			outcome = "TIMEOUT";
			end_state = true;
		}
		experimentLogFile << ros::Time::now().toNSec() - start_time_nSec << "," << veh_x_ << "," << veh_y_ << "," << veh_heading_ << "," << veh_speed_ << "," << distance_traveled << "," << throttle << "," << steering << "," << braking << "\n";
		sum_throttle = sum_throttle + throttle;

		nav_msgs::Odometry true_odom = mavs_ros_utils::CopyFromMavsVehicleState(veh_state);

		if (render_debug && nsteps%10==0){
			camera.SetPose(veh_state);
			camera.Update(&env, 0.033);
			camera.Display();
		}

		//piksi update
		piksi.SetPose(veh_state);
		piksi.Update(&env, 0.01);
		if (elapsed_time > 1.0){
			mavs::Odometry mavs_odom = piksi.GetOdometryMessage();
			nav_msgs::Odometry odom_msg;
			mavs_ros_utils::CopyFromMavsOdometry(odom_msg, mavs_odom);
			odom_msg.header.stamp = ros::Time::now();
			true_odom.header.stamp = ros::Time::now();
			odom_msg.header.frame_id = "odom";
			true_odom.header.frame_id = "odom";
			rtk_pub.publish(odom_msg);
		}

		true_odom.header.stamp = ros::Time::now();

		odom_true.publish(true_odom);

		//clock update
		if (use_sim_time){
			ros::Time now(elapsed_time);
			rosgraph_msgs::Clock clock_msg;
			clock_msg.clock = now;
			clock_pub.publish(clock_msg);
		}
		else{
			rate.sleep();
		}
		elapsed_time += dt;
		nsteps++;
		ros::spinOnce();
	} //while ros OK

	double tot_time = ros::Time::now().toSec() - start_time;
	if (use_sim_time)
		tot_time = elapsed_time;

	experimentLogFile.close();
	std::ofstream experimentResultsFile;
	experimentResultsFile.open("/cavs/projects/ARC/Project1.31/users/dwc2/2021casestudy/data/results/"+condition+"-case-study-results.csv", std::ios::out | std::ios::app);
	experimentResultsFile << ros::Time::now().toSec() << ",";
    experimentResultsFile << trial_name << "," << soil_strength << "," << surface_type << "," << rain_rate << ",";
	experimentResultsFile << x_init << "," << y_init << "," << prev_x << "," << prev_y << ",";
	experimentResultsFile << tot_time << "," << elapsed_time << "," << distance_traveled << "," << outcome << "," << sum_throttle << "\n";
	
	return 0;
}
