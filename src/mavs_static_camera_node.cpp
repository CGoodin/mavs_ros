//ros includes
#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/Image.h"
#include "stereo_msgs/DisparityImage.h"
#include "sensor_msgs/CameraInfo.h"
// local includes
#include "mavs_ros_utils.h"
// mavs includes
#include "sensors/sensor.h"
#include "sensors/mavs_sensors.h"
#include "sensors/camera/path_tracer.h"
#include "raytracers/embree_tracer/embree_tracer.h"
#include "mavs_core/math/utils.h"
#include "vehicles/rp3d_veh/mavs_rp3d_veh.h"
#include "mavs_core/data_path.h"

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
	ros::init(argc, argv, "mavs_static_camera_node");
	ros::NodeHandle n;

	ros::Publisher camera_pub = n.advertise<sensor_msgs::Image>("mavs_ros/image", 1);

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

	bool display_image = false;
	if (ros::param::has("~display_image")){
		ros::param::get("~display_image", display_image);
	}
	float update_rate_hz = 20.0f;
	if (ros::param::has("~update_rate_hz")){
		ros::param::get("~update_rate_hz",update_rate_hz);
	}

	float pos_x = 1.0f;
	if (ros::param::has("~pos_x")){
		ros::param::get("~pos_x",pos_x);
	}
	float pos_y = 0.0f;
	if (ros::param::has("~pos_y")){
		ros::param::get("~pos_y",pos_y);
	}
	float pos_z = 1.5f;
	if (ros::param::has("~pos_z")){
		ros::param::get("~pos_z",pos_z);
	}
	float qw = 1.0;
	if (ros::param::has("~qw")){
		ros::param::get("~qw",qw);
	}
	float qx = 1.0;
	if (ros::param::has("~qx")){
		ros::param::get("~qx",qx);
	}
	float qy = 1.0;
	if (ros::param::has("~qy")){
		ros::param::get("~qy",qy);
	}
	float qz = 1.0;
	if (ros::param::has("~qz")){
		ros::param::get("~qz",qz);
	}
	glm::vec3 position(pos_x, pos_y, pos_z);
	glm::quat orientation(qw, qx, qy, qz);

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

	mavs::sensor::camera::RgbCamera camera;
	camera.Initialize(num_horizontal_pix,num_vertical_pix, horizontal_pixdim, vertical_pixdim, focal_length);

	double dt = 1.0f/update_rate_hz;
	ros::Rate rate(1.0 / dt);
	while (ros::ok()){

		if ((int)poses_map.size()==num_veh || num_veh==1){

			// consolidate all the animation poses
			geometry_msgs::PoseArray anim_poses;
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
			
			// update the camera
			camera.SetPose(position, orientation);
			camera.Update(&env, dt);
		
			// convert to a ROS image
			mavs::Image mavs_img;
			sensor_msgs::Image img;
			mavs_ros_utils::CopyFromMavsImage(img, mavs_img);

			// publish the image
			img.header.stamp = ros::Time::now();
			img.header.frame_id = "static_camera";
			camera_pub.publish(img);

			if (display_image){
				camera.Display();
			}
		}
		rate.sleep();
		ros::spinOnce();
	} //while ros OK

	return 0;
}
