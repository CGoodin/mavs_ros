// c++ includes
#include <omp.h>
#include <unistd.h>
//ros includes
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Twist.h"
#include "rosgraph_msgs/Clock.h"
#include "sensor_msgs/Imu.h"
#include <std_msgs/Int64.h>
#include <tf/transform_broadcaster.h>
// local includes
#include "mavs_ros_utils.h"
// mavs includes
#include "mavs_core/math/utils.h"
#include "vehicles/rp3d_veh/mavs_rp3d_veh.h"
#include "raytracers/embree_tracer/embree_tracer.h"
#include "sensors/mavs_sensors.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"

float auto_throttle = 0.0f;
float auto_steering = 0.0f;
float auto_braking = 0.0f;
float human_throttle = 0.0;
float human_steering = 0.0;
float human_braking = 0.0;
void TwistCallback(const geometry_msgs::Twist::ConstPtr &rcv_msg){
	auto_throttle = rcv_msg->linear.x;
	auto_braking = rcv_msg->linear.y;
	auto_steering = rcv_msg->angular.z;
}

int main(int argc, char **argv){
	//- Create the node and subscribers ---//
	ros::init(argc, argv, "mavs_vehicle_node");
	ros::NodeHandle n;

	ros::Publisher clock_pub;
	ros::Subscriber twist_sub = n.subscribe("mavs_ros/cmd_vel", 1, TwistCallback);

	ros::Publisher odom_true = n.advertise<nav_msgs::Odometry>("mavs_ros/odometry_true", 10);
	ros::Publisher anim_poses_pub = n.advertise<geometry_msgs::PoseArray>("mavs_ros/anim_poses", 10);
	ros::Publisher rtk_pub = n.advertise<nav_msgs::Odometry>("mavs_ros/odometry", 10);
	ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("mavs_ros/imu", 10);
	ros::Publisher camera_pub = n.advertise<sensor_msgs::Image>("mavs_ros/vehicle_debug_image", 1);

	//--- get parameters ---//
	bool use_sim_time = false;
	if (n.hasParam("use_sim_time")){
		n.getParam("use_sim_time", use_sim_time);
	}
	if (use_sim_time){
		clock_pub = n.advertise<rosgraph_msgs::Clock>("clock", 1);
	}

	std::string scene_file;
	if (ros::param::has("~scene_file")){
		ros::param::get("~scene_file", scene_file);
	}
	else{
		std::cerr << "ERROR: No scene file listed " << std::endl;
	}

	float soil_strength = 250.0f;
	if (ros::param::has("~soil_strength")){
		ros::param::get("~soil_strength", soil_strength);
	}
	std::string surface_type = "dry";
	if (ros::param::has("~surface_type")){
		ros::param::get("~surface_type", surface_type);
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

	bool publish_imu = false;
	if (ros::param::has("~publish_imu")){
		ros::param::get("~publish_imu", publish_imu);
	}

	bool use_human_driver = false;
	if (ros::param::has("~use_human_driver")){
		ros::param::get("~use_human_driver", use_human_driver);
	}

	bool shared_control = false;
	if (ros::param::has("~shared_control")){
		ros::param::get("~shared_control", shared_control);
	}

	std::string vehicle_id = "mavs_vehicle";
	if (ros::param::has("~vehicle_id")){
		ros::param::get("~vehicle_id", vehicle_id);
	}
	std::string sensor_name="vehicle_camera";
	if (ros::param::has("~sensor_name")){
		ros::param::get("~sensor_name", sensor_name);
	}

	float auto_frac = 0.0f;
	float human_frac = 1.0f;
	if (ros::param::has("~auto_frac")){
		ros::param::get("~auto_frac", auto_frac);
		auto_frac = std::min(1.0f,std::max(0.0f, auto_frac));
		human_frac = 1.0f-auto_frac;
	}

	mavs::sensor::imu::ImuSimple imu;
	if (ros::param::has("~imu_input_file")){
		std::string imu_input_file;
		ros::param::get("~imu_input_file", imu_input_file);
		imu.Load(imu_input_file);
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
	mavs_veh.Update(&env, 0.0f, 0.0f, 0.0f, 0.00001);

	mavs::sensor::camera::RgbCamera camera;
	camera.Initialize(384, 384, 0.0035, 0.0035, 0.0035);
	camera.SetName(sensor_name);
	camera.SetRenderShadows(false);
	glm::vec3 cam_offset(-10.0, 0.0, 2.0);
	camera.SetRelativePose(cam_offset, relor);

	double dt = 0.01;
	ros::Rate rate(1.0 / dt);
	int nsteps = 0;
	double elapsed_time = 0.0;

	double start_time = ros::Time::now().toSec();
	while (ros::ok()){
		//vehicle state update
		
		if (use_human_driver || shared_control){	
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
		}

		float throttle, steering, braking;
		if (shared_control){
			if (human_braking!=0.0f || auto_braking!=0.0f){
				throttle = 0.0f;
				braking = auto_frac*auto_braking + human_frac*human_braking;
			}
			else{
				throttle = auto_frac*auto_throttle + human_frac*human_throttle;
				braking = 0.0f;
			}
			steering = auto_frac*auto_steering + human_frac*human_steering;
		}
		else if (use_human_driver){
			throttle = human_throttle;
			steering = human_steering;
			braking = human_braking;
		}
		else{
			throttle = auto_throttle;
			steering = auto_steering;
			braking = auto_braking;
		}

		mavs_veh.Update(&env, throttle, steering, -braking, dt);
		mavs::VehicleState veh_state = mavs_veh.GetState();

		nav_msgs::Odometry true_odom = mavs_ros_utils::CopyFromMavsVehicleState(veh_state);

		if (render_debug && nsteps%10==0){
			camera.SetPose(veh_state);
			camera.Update(&env, 0.033);
			//camera.Display();
			mavs::Image mavs_img = camera.GetRosImage();
			sensor_msgs::Image img;
			mavs_ros_utils::CopyFromMavsImage(img, mavs_img);
			img.header.stamp = ros::Time::now();
			img.header.frame_id = "odom";
			camera_pub.publish(img);
		}

		// imu update
		if (publish_imu){
			imu.SetPose(veh_state);
			imu.Update(&env, 0.01);
			glm::vec3 accel = imu.GetAcceleration();
			glm::vec3 angvel = imu.GetAngularVelocity();
			sensor_msgs::Imu imu_msg;
			imu_msg.header.stamp = ros::Time::now();
			imu_msg.header.frame_id = "odom";
			imu_msg.angular_velocity.x = angvel.x;
			imu_msg.angular_velocity.y = angvel.y;
			imu_msg.angular_velocity.z = angvel.z;
			imu_msg.linear_acceleration.x = accel.x;
			imu_msg.linear_acceleration.y = accel.y;
			imu_msg.linear_acceleration.z = accel.z;
			imu_pub.publish(imu_msg);
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
			odom_msg.child_frame_id = vehicle_id;
			rtk_pub.publish(odom_msg);
		}

		true_odom.header.stamp = ros::Time::now();

		odom_true.publish(true_odom);

		// publish the pose of all the animated objects associated with the vehicle
		geometry_msgs::PoseArray anim_poses;
		geometry_msgs::Pose vpose;
		vpose.position = true_odom.pose.pose.position;
		vpose.orientation = true_odom.pose.pose.orientation;
		anim_poses.poses.push_back(vpose);
		for (int i = 0; i < mavs_veh.GetNumTires(); i++){
			glm::vec3 tpos = mavs_veh.GetTirePosition(i);
			glm::quat tori = mavs_veh.GetTireOrientation(i);
			geometry_msgs::Pose tpose;
			tpose.position.x = tpos.x;
			tpose.position.y = tpos.y;
			tpose.position.z = tpos.z;
			tpose.orientation.w = tori.w;
			tpose.orientation.x = tori.x;
			tpose.orientation.y = tori.y;
			tpose.orientation.z = tori.z;
			anim_poses.poses.push_back(tpose);
		}
		anim_poses.header.frame_id = vehicle_id;
		anim_poses.header.stamp = ros::Time::now();
		anim_poses_pub.publish(anim_poses);

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

	return 0;
}
