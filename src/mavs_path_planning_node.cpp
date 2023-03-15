// c++ includes
#include <fstream>
//ros includes
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
// package includes
#include "mavs_ros_utils.h"
#include "astar.h"
// mavs includes
#include "raytracers/embree_tracer/embree_tracer.h"
#include "sensors/occupancy_grid_detector/occupancy_grid_detector.h"

nav_msgs::Odometry odom;
bool rcvd_odom = false;

void OdomCallback(const nav_msgs::Odometry::ConstPtr& rcv_odom){
	odom = *rcv_odom;
	rcvd_odom = true;
}

int main(int argc, char **argv){
	//- Create the node and subscribers ---//
	ros::init(argc, argv, "mavs_path_planning_node");
	ros::NodeHandle n;

	ros::Subscriber odom_sub = n.subscribe("mavs_ros/odometry_true", 1, OdomCallback);

	ros::Publisher path_pub = n.advertise<nav_msgs::Path>("mavs_ros/global_path", 10);
	ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("mavs_ros/cost_map", 10);
	ros::Publisher map_vis_pub = n.advertise<nav_msgs::OccupancyGrid>("mavs_ros/cost_map_vis", 10);

	std::string scene_file;
	if (ros::param::has("~scene_file")){
		ros::param::get("~scene_file", scene_file);
	}
	else{
		std::cerr << "ERROR: No scene file listed " << std::endl;
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


	float goal_x = 10.0f;
	if (ros::param::has("~goal_x")){
		ros::param::get("~goal_x", goal_x);
	}
	float goal_y = 0.0f;
	if (ros::param::has("~goal_y")){
		ros::param::get("~goal_y", goal_y);
	}

	std::vector<float> goal;
  	goal.resize(2, 0.0f);
	goal[0] = goal_x; goal[1] = goal_y;

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
	mavs::OccupancyGrid mavs_grid = detector.GetGrid();
	nav_msgs::OccupancyGrid grid, seg_grid;
	mavs_ros_utils::CopyFromMavsGrid(grid,mavs_grid);


	mavs_ros::Astar astar_planner;

	double dt = 1.0/50.0;
	ros::Rate rate(1.0 / dt);

	bool path_found = false;
	nav_msgs::Path ros_path;

	while (ros::ok()){


		if (rcvd_odom & !path_found){
			std::vector<float> pos;
      		pos.push_back(odom.pose.pose.position.x);
      		pos.push_back(odom.pose.pose.position.y);
			std::vector<std::vector<float>> path = astar_planner.PlanPath(&grid, &seg_grid, goal, pos);
      		ros_path.header.frame_id = "map";
      		ros_path.poses.clear();
      		for (int32_t i = 0; i < path.size(); i++){
        		geometry_msgs::PoseStamped pose;
        		pose.pose.position.x = static_cast<float>(path[i][0]);
        		pose.pose.position.y = static_cast<float>(path[i][1]);
       			pose.pose.position.z = 0.0f;
        		pose.pose.orientation.w = 1.0f;
        		pose.pose.orientation.x = 0.0f;
        		pose.pose.orientation.y = 0.0f;
        		pose.pose.orientation.z = 0.0f;
        		ros_path.poses.push_back(pose);
			}
			path_found = true;
		}

		if (path_found)path_pub.publish(ros_path);
		map_pub.publish(grid);
		map_vis_pub.publish(grid);

		rate.sleep();
		ros::spinOnce();
	} //while ros OK

	return 0;
}
