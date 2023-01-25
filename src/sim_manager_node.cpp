// c++ includes
#include <fstream>
//ros includes
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
// package includes
#include "mavs_ros_utils.h"
// mavs includes
#include "raytracers/embree_tracer/embree_tracer.h"
#include "sensors/occupancy_grid_detector/occupancy_grid_detector.h"

nav_msgs::Odometry odom;
bool rcvd_odom = false;

void OdomCallback(const nav_msgs::Odometry::ConstPtr& rcv_odom){
	odom = *rcv_odom;
	rcvd_odom = true;
}

bool Collision(mavs::OccupancyGrid *grid, float px_c, float py_c, float vl, float vw, int thresh){

	
	
	float heading = mavs_ros_utils::GetHeadingFromOrientation(odom.pose.pose.orientation);
	float ltx = cosf(heading);
	float lty = sinf(heading);
	float lsx = -sinf(heading);
	float lsy = cosf(heading);
	float len = 0.5f*vl;
	float wid = 0.5f*vw;
	float px = px_c - ltx*len - lsx*wid;
	float py = py_c - lty*len - lsy*wid;

	float dx = 0.0f;
	float step = grid->info.resolution;
	bool collis = false;

	while (dx<=vl){
		float dy = 0.0f;
		while (dy<=vw){
			float pxx = px + dx*ltx + dy*lsx;
			float pyy = py + dx*lty + dy*lsy;
			
			int i = (int)floor((pxx - grid->info.origin.position.x)/grid->info.resolution);
			int j = (int)floor((pyy - grid->info.origin.position.y)/grid->info.resolution);

			if (i<grid->info.width && i>=0 && j<grid->info.height && j>=0){
				int n = i*grid->info.width + j;
				
				if (grid->data[n]>thresh){
					collis = true;
					break;
				}
			}
			dy += step;
		}
		dx += step;
	}
	return collis;
}

int main(int argc, char **argv){
	//- Create the node and subscribers ---//
	ros::init(argc, argv, "sim_manager_node");
	ros::NodeHandle n;

	ros::Subscriber odom_sub = n.subscribe("mavs_ros/odometry_true", 1, OdomCallback);

	std::string scene_file;
	if (ros::param::has("~scene_file")){
		ros::param::get("~scene_file", scene_file);
	}
	else{
		std::cerr << "ERROR: No scene file listed " << std::endl;
	}

	float timeout = 1000000.0f;
	if (ros::param::has("~timeout")){
		ros::param::get("~timeout", timeout);
	}
	float upper_x_limit = 100.0f;
	if (ros::param::has("~upper_x_limit")){
		ros::param::get("~upper_x_limit", upper_x_limit);
	}
	float upper_y_limit = 100.0f;
	if (ros::param::has("~upper_y_limit")){
		ros::param::get("~upper_y_limit", upper_y_limit);
	}
	float lower_x_limit = -100.0f;
	if (ros::param::has("~lower_x_limit")){
		ros::param::get("~lower_x_limit", lower_x_limit);
	}
	float lower_y_limit = -100.0f;
	if (ros::param::has("~lower_y_limit")){
		ros::param::get("~lower_y_limit", lower_y_limit);
	}
	float min_obs_height = 0.5f;
	if (ros::param::has("~min_obs_height")){
		ros::param::get("~min_obs_height", min_obs_height);
	}
	float truth_map_res = 0.25f;
	if (ros::param::has("~truth_map_res")){
		ros::param::get("~truth_map_res", truth_map_res);
	}
	float vehicle_length = 3.55f;
	if (ros::param::has("~vehicle_length")){
		ros::param::get("~vehicle_length", vehicle_length);
	}
	float vehicle_width = 1.55f;
	if (ros::param::has("~vehicle_width")){
		ros::param::get("~vehicle_width", vehicle_width);
	}
	bool display_map = false;
	if (ros::param::has("~display_map")){
		ros::param::get("~display_map", display_map);
	}
	bool save_trajectory = false;
	if (ros::param::has("~save_trajectory")){
		ros::param::get("~save_trajectory", save_trajectory);
	}
	int obstacle_slope_thresh = 35;
	if (ros::param::has("~obstacle_slope_thresh")){
		ros::param::get("~obstacle_slope_thresh", obstacle_slope_thresh);
	}

	mavs::environment::Environment env;
	mavs::raytracer::embree::EmbreeTracer scene;
	scene.Load(scene_file); 
	scene.TurnOffLabeling();
	env.SetRaytracer(&scene);
	mavs::sensor::ogd::OccupancyGridDetector detector;
	float px = 0.5f*(upper_x_limit + lower_x_limit);
	float py = 0.5f*(upper_y_limit + lower_y_limit);
	detector.Initialize(upper_x_limit-lower_x_limit, upper_y_limit-lower_y_limit, truth_map_res);
	detector.SetPose(glm::vec3(px,py,0.0f),glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
	detector.SetMaxHeight(min_obs_height);
	detector.SetNumIgnore(0);
	detector.Update(&env, 0.1);
	mavs::OccupancyGrid occ_grid = detector.GetGrid();
	if (display_map)detector.Display();

	double dt = 1.0/50.0;
	ros::Rate rate(1.0 / dt);
	double elapsed_time = 0.0;
	double start_time = ros::Time::now().toSec();
	bool failed = false;
	std::ofstream sout("sim_log_result.txt");
	std::ofstream fout;
	if (save_trajectory){
		fout.open("trajectory.txt");
		fout <<"Elapsed_Time Pos_x Pos_y Speed"<<std::endl;
	}
	while (ros::ok()){

		elapsed_time = ros::Time::now().toSec() - start_time;

		if (elapsed_time>timeout){
			sout<<"TIMEOUT"<<std::endl;
			failed = true;
			break;
		}

		if (rcvd_odom){
			float px = odom.pose.pose.position.x;
			float py = odom.pose.pose.position.y;
			float vx = odom.twist.twist.linear.x;
			float vy = odom.twist.twist.linear.y;
			float speed = sqrtf(vx*vx + vy*vy);
			if (save_trajectory) fout<<elapsed_time<<" "<<px<<" "<<py<<" "<<speed<<std::endl;
			
			if (px<lower_x_limit || px>upper_x_limit || py<lower_y_limit || py>upper_y_limit){
				sout<<"OUT OF BOUNDS"<<std::endl;
				failed = true;
				break;
			}	
			if (Collision(&occ_grid, px, py, vehicle_length, vehicle_width, obstacle_slope_thresh)){
				sout<<"COLLISION"<<std::endl;
				failed = true;
				break;
			}
		}

		rate.sleep();
		ros::spinOnce();
	} //while ros OK

	if (!failed)sout<<"SUCCESS"<<std::endl;
	sout.close();
	if (save_trajectory) fout.close();
	return 0;
}
