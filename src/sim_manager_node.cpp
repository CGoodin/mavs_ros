// c++ includes
#include <fstream>
//ros includes
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
// mavs includes
#include "raytracers/embree_tracer/embree_tracer.h"
#include "sensors/occupancy_grid_detector/occupancy_grid_detector.h"

nav_msgs::Odometry odom;
bool rcvd_odom = false;
void OdomCallback(const nav_msgs::Odometry::ConstPtr& rcv_odom){
	odom = *rcv_odom;
	rcvd_odom = true;
}

bool Collision(mavs::OccupancyGrid *grid, float px, float py){
	int i = (int)floor((px - grid->info.origin.position.x)/grid->info.resolution);
	int j = (int)floor((py - grid->info.origin.position.y)/grid->info.resolution);
	
	//int n = i*grid->info.height + j;
	int n = j*grid->info.width + i;
	std::cout<<"Checking collision at "<<i<<" "<<j<<" "<<grid->info.width<<" "<<grid->info.height<<" "<<n<<" "<<grid->data[n]<<std::endl;
	
	if (i>=grid->info.width || i<0 || j>=grid->info.height || j<0){
		// off the map, register as collision
		return true;
	}
	
	if (grid->data[n]>0){
		return true;
	}
	else{
		return false;
	}
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

	mavs::environment::Environment env;
	mavs::raytracer::embree::EmbreeTracer scene;
	scene.Load(scene_file); 
	scene.TurnOffLabeling();
	env.SetRaytracer(&scene);
	mavs::sensor::ogd::OccupancyGridDetector detector;
	float px = 0.5f*(upper_x_limit + lower_x_limit);
	float py = 0.5f*(upper_y_limit + lower_y_limit);
	detector.Initialize(upper_x_limit-lower_x_limit, upper_y_limit-lower_y_limit, 0.1f);
	detector.SetPose(glm::vec3(px,py,0.0f),glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
	detector.Update(&env, 0.1);
	mavs::OccupancyGrid occ_grid = detector.GetGrid();

	double dt = 0.1;
	ros::Rate rate(1.0 / dt);
	double elapsed_time = 0.0;
	double start_time = ros::Time::now().toSec();
	bool failed = false;
	std::ofstream sout("sim_log_result.txt");
	std::ofstream fout("trajectory.txt");
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
			fout<<elapsed_time<<" "<<px<<" "<<py<<" "<<speed<<std::endl;
			
			if (px<lower_x_limit || px>upper_x_limit || py<lower_y_limit || py>upper_y_limit){
				sout<<"OUT OF BOUNDS"<<std::endl;
				failed = true;
				break;
			}	
			if (Collision(&occ_grid, px, py)){
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
	fout.close();
	return 0;
}
