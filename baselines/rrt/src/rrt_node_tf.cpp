/******************************************************************************
Copyright (c) 2020 Georgia Instititue of Technology 
Copyright (c) 2020 Tsinghua University
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
Author: Jianming TONG (jtong45@gatech.edu)

Modified for use with the dmce_sim environment
by: Sean Bone (seanbone.ch), 2022, ETH Zürich
*******************************************************************************/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "stdint.h"
#include "functions.h"
#include "mtrand.h"
#include<math.h>

#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_listener.h>

#include <actionlib/client/simple_action_client.h>
// #include "cartographer_ros_msgs/TrajectoryQuery.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include <mutex>

// Headers from dmce simulation
#include "dmce_core/TypeDefs.hpp"
#include "dmce_msgs/RobotPlan.h"

#define THRESHOLD_TRANSFORM 10 // was 0.5
std::mutex _mt;

// -----------------------------------  functions
double mapValue(const nav_msgs::OccupancyGrid & mapDataIn, const geometry_msgs::PointStamped & point){
    _mt.lock();
    double tempResult;
    int idx;
    idx = (floor((point.point.y-mapDataIn.info.origin.position.y)/mapDataIn.info.resolution)*mapDataIn.info.width)+(floor((point.point.x-mapDataIn.info.origin.position.x)/mapDataIn.info.resolution));
    tempResult = mapDataIn.data[idx];
    _mt.unlock();
    return tempResult;
}

double norm2Dpoints(const double& point1_x, const double& point1_y, const geometry_msgs::PointStamped & point2)
{
    _mt.lock();
    double tempResult;
    tempResult= pow(	(pow((point2.point.x-point1_x),2)+pow((point2.point.y-point1_y),2))	,0.5);
    _mt.unlock();
    return tempResult;
}


double informationGain(const nav_msgs::OccupancyGrid& mapDataIn, const geometry_msgs::PointStamped & point, const double r){
    _mt.lock();
    double infoGainValue = 0, tempResult;
    int index, init_index, last_index, length, start, end, r_region, limit;
    double x = point.point.x, y = point.point.y;

    index = (floor((y-mapDataIn.info.origin.position.y)/mapDataIn.info.resolution)*mapDataIn.info.width)+(floor((x-mapDataIn.info.origin.position.x)/mapDataIn.info.resolution));
                  
    r_region = int(r/mapDataIn.info.resolution);
    init_index = index-r_region*(mapDataIn.info.width+1);
    last_index = index+r_region*(mapDataIn.info.width+1);
    length     = 2*r_region;
    start      = int(init_index);
    end        = start + int(length);

    if(last_index < mapDataIn.data.size()){
        for(int i = 0; i < 2 * r_region + 1; i++){
            
            int deltaIdx_y = abs(r_region - i);
            int deltaIdx_x = floor(std::sqrt(r_region*r_region - deltaIdx_y*deltaIdx_y));
            int temp = r_region-deltaIdx_x;
            int startIdxCircle = start + temp;
            int endIdxCircle   = end   - temp;
            
            for(int j = startIdxCircle; j < endIdxCircle; j++){
                switch(mapDataIn.data[j]){
                    case  -1: {infoGainValue++; break;}
                    case 100: {infoGainValue--; break;}
                    default: {break;}
                }
            }
            start += mapDataIn.info.width;
            end   += mapDataIn.info.width;
        }
    }
    else{
        for(int i = 0; i < 2 * r_region + 1; i++){

            int deltaIdx_y = abs(r_region - i);
            int deltaIdx_x = floor(std::sqrt(r_region*r_region - deltaIdx_y*deltaIdx_y));
            int temp = r_region-deltaIdx_x;
            int startIdxCircle = start + temp;
            int endIdxCircle   = end   - temp;
            
            for(int j = startIdxCircle; j < end; j++){
                limit = ((start/mapDataIn.info.width) + 2)*mapDataIn.info.width;  // part of rectangle is outside the map
                    if(j >= 0 && j < limit && j < mapDataIn.data.size()){
                    switch(mapDataIn.data[j]){
                        case  -1: {infoGainValue++; break;}
                        case 100: {infoGainValue--; break;}
                        default: {break;}
                    }
                }
            }
            start += mapDataIn.info.width;
            end   += mapDataIn.info.width;
        }
    }
    tempResult = infoGainValue*(pow(mapDataIn.info.resolution,2));
    _mt.unlock();
    return tempResult;
}



double informationRectangleGain(const nav_msgs::OccupancyGrid& mapDataIn, const geometry_msgs::PointStamped & point, const double r){
    _mt.lock();
    double infoGainValue = 0, tempResult;
    int index, init_index, last_index, length, start, end, r_region, limit;
    double x = point.point.x, y = point.point.y;

    index = (floor((y-mapDataIn.info.origin.position.y)/mapDataIn.info.resolution)*mapDataIn.info.width)+(floor((x-mapDataIn.info.origin.position.x)/mapDataIn.info.resolution));
                  
    r_region = int(r/mapDataIn.info.resolution);
    init_index = index-r_region*(mapDataIn.info.width+1);
    last_index = index+r_region*(mapDataIn.info.width+1);
    length     = 2*r_region;
    start      = int(init_index);
    end        = start + int(length);

    if(last_index < mapDataIn.data.size()){
        for(int i = 0; i < 2 * r_region + 1; i++){
            for(int j = start; j < end; j++){
                switch(mapDataIn.data[j]){
                    case  -1: {infoGainValue++; break;}
                    case 100: {infoGainValue--; break;}
                    default: {break;}
                }
            }
            start += mapDataIn.info.width;
            end   += mapDataIn.info.width;
        }
    }
    else{
        for(int i = 0; i < 2 * r_region + 1; i++){
            for(int j = start; j < end; j++){
                limit = ((start/mapDataIn.info.width) + 2)*mapDataIn.info.width;  // part of rectangle is outside the map
                    if(j >= 0 && j < limit && j < mapDataIn.data.size()){
                    switch(mapDataIn.data[j]){
                        case  -1: {infoGainValue++; break;}
                        case 100: {infoGainValue--; break;}
                        default: {break;}
                    }
                }
            }
            start += mapDataIn.info.width;
            end   += mapDataIn.info.width;
        }
    }
    tempResult = infoGainValue*(pow(mapDataIn.info.resolution,2));
    _mt.unlock();
    return tempResult;
}


// -----------------------------------  global variables
nav_msgs::OccupancyGrid     mapData, costmapData;
geometry_msgs::PointStamped clickedpoint;
visualization_msgs::Marker  points,line;

// -----------------------------------  for genrating random numbers
rdm r; 

// -----------------------------------  Subscribers callback functions-
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    _mt.lock();
	mapData=*msg;
    // Added to remap unknown values from dmce maps
    for (unsigned int i = 0; i < mapData.data.size(); i++) {
        if (mapData.data[i] > 45 && mapData.data[i] < 55) {
            mapData.data[i] = -1;
        }
    }
    // std::cout << "assigner receives map" << std::endl;
    _mt.unlock();
}

void costmapMergedCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    _mt.lock();
	costmapData=*msg;
    // Added to remap unknown values from dmce maps
    for (unsigned int i = 0; i < costmapData.data.size(); i++) {
        if (costmapData.data[i] > 45 && costmapData.data[i] < 55) {
            costmapData.data[i] = -1;
        }
    }
    // std::cout << "assigner receives costmap" << std::endl;
    _mt.unlock();
    
}

geometry_msgs::Point p;  


void rvizCallBack(const geometry_msgs::PointStamped::ConstPtr& msg)
{ 
	p.x=msg->point.x;
	p.y=msg->point.y;
	p.z=msg->point.z;
	points.points.push_back(p);
}


int main(int argc, char **argv)
{
	// -------------------------------------random number generator
 	// this is an example of initializing by an array
	// you may use MTRand(seed) with any 32bit integer
	// as a seed for a simpler initialization
	unsigned long init[4] = {0x123, 0x234, 0x345, 0x456}, length = 7;
	MTRand_int32 irand(init, length); // 32-bit int generator
	MTRand drand; // double in [0, 1) generator, already init

	// -------------------------------------initialize variables
	std::string map_topic, costmap_topic, base_frame_topic, frontier_topic;
    std::string robot_frame, robot_base_frame, robot_ano_frame_preffix, robot_ano_frame_suffix;
    std::string trajectory_query_name, output_file, output_map_file;
	int rateHz, detectFrontierNum = 0;
	double costmap_pixel_threshold, info_radius;
    float hysteresis_radius, hysteresis_gain;
    geometry_msgs::PointStamped exploration_goal, goalRobotFrame, previous_goal_map_frame;
    float xdim, ydim, resolution, centerX, centerY, map_lengthX, map_lengthY, eta, range;
    // -------------------------------------initialized the variables which are in need in exploration
	std::vector<geometry_msgs::PointStamped> frontiers;
	int i=0;
	float xr,yr;
	std::vector<float> x_rand,x_nearest,x_new;
    double previous_goal_infoGain;
    int    n_robot, this_robot_idx;
    
    // -------------------------------------initialize the robot node
	ros::init(argc, argv, "rrt_exploration_node");
	ros::NodeHandle nh;
	std::string nodename, ns;
	nodename=ros::this_node::getName();
	
	// -------------------------------------fetch params
	ros::param::param<std::string>(nodename+"/namespace", ns, "robot1/");
	ros::param::param<std::string>(nodename+"/map_topic", map_topic, "robot1/map"); 
    ros::param::param<std::string>(nodename+"/costmap_topic", costmap_topic, "robot1/move_base/global_costmap/costmap");
    ros::param::param<std::string>(nodename+"/robot_frame", robot_frame, "robot1/map");
    ros::param::param<std::string>(nodename+"/robot_base_frame", robot_base_frame, "robot1/base_link");
    ros::param::param<std::string>(nodename+"/robot_ano_frame_preffix", robot_ano_frame_preffix, "robot");
    ros::param::param<std::string>(nodename+"/robot_ano_frame_suffix", robot_ano_frame_suffix, "/map");
    ros::param::param<std::string>(nodename+"/trajectory_query_name", trajectory_query_name, "robot1/trajectory_query");
    ros::param::param<std::string>(nodename+"/output_file", output_file, "/home/jimmy/work/ROS_work/multirobot_github_ws/src/rrt_exploration_real_two_cars/robot1_rrt_trajectory.txt");
    ros::param::param<std::string>(nodename+"/output_map_file", output_map_file, "/home/jimmy/work/ROS_work/multirobot_github_ws/src/rrt_exploration_real_two_cars/robot1_rrt_explored_map.txt");

    ros::param::param<int>(nodename+"/n_robot", n_robot, 1);
    ros::param::param<int>(nodename+"/this_robot_idx", this_robot_idx, 1);

    ros::param::param<int>(nodename+"/rate", rateHz, 20);
    ros::param::param<double>(nodename+"/info_radius", info_radius, 1.0);
    ros::param::param<double>(nodename+"/costmap_pixel_threshold", costmap_pixel_threshold, 30);
	ros::param::param<float>(nodename+"/eta", eta, 0.5);

	ros::param::param<float>(nodename+"/hysteresis_radius", hysteresis_radius, 3.0);
	ros::param::param<float>(nodename+"/hysteresis_gain", hysteresis_gain, 2.0);

	ros::Rate rate(rateHz);

    // -------------------------------------Transforms between all robots initilization;
    std::vector<tf::StampedTransform> transform_vec;
    for (int i = 0; i < n_robot; i++){
        tf::StampedTransform trans_ ;
        tf::Vector3 translation;
        translation.setX(0);
        translation.setY(0);
        translation.setZ(0);
        translation.setW(0);

        trans_.setOrigin(translation);
        transform_vec.push_back(trans_);
    }

    // -------------------------------------Initialize all robots' frame;
    std::vector<std::string> robots_frame;

    for (int i = 0; i < n_robot; i++){

        std::stringstream ss;              
        ss << robot_ano_frame_preffix;
        ss << i+1;
        ss << robot_ano_frame_suffix;

        robots_frame.push_back(ss.str());
    }

	// -------------------------------------subscribe the map topics & clicked points
	tf::TransformListener listener;
	ros::Subscriber sub       = nh.subscribe(map_topic, 100 ,mapCallBack);	
    ros::Subscriber costMapSub= nh.subscribe<nav_msgs::OccupancyGrid>(costmap_topic, 10, costmapMergedCallBack);
	
    ros::Subscriber rviz_sub  = nh.subscribe<geometry_msgs::PointStamped>("/clicked_point", 10, rvizCallBack);

	// -------------------------------------publish the detected points for following processing & display
	ros::Publisher shapes_pub   = nh.advertise<visualization_msgs::Marker>(nodename+"_shapes", 10);
	ros::Publisher pub_frontier = nh.advertise<visualization_msgs::Marker>(nodename+"_detected_frontiers", 10);

    // Publisher for sending plan to dmce simulation
    ros::Publisher dmce_goalPublisher = nh.advertise<dmce_msgs::RobotPlan>("rrt_plan", 10);

	// -------------------------------------wait until map is received
    std::cout << ns << "wait for map "<< std::endl;
	while (mapData.header.seq < 1  or  mapData.data.size() < 1 ){  ros::spinOnce();  ros::Duration(0.1).sleep(); }
	std::cout << ns << "wait for costmap "<< std::endl;
    while ( costmapData.data.size()<1)  {  ros::spinOnce();  ros::Duration(0.1).sleep();}

	// -------------------------------------initilize the visualized points & lines  
	points.header.frame_id  = mapData.header.frame_id;
	points.header.stamp     = ros::Time(0);
	points.type 			= points.POINTS;
	points.action           = points.ADD;
	points.pose.orientation.w =1.0;
	points.scale.x 			= 0.3; 
	points.scale.y			= 0.3; 
	points.color.r 			= 1.0;   // 255.0/255.0;
	points.color.g 			= 0.0;   // 0.0/255.0;
	points.color.b 			= 0.0;   // 0.0/255.0;
	points.color.a			= 1.0;
	points.lifetime         = ros::Duration();

	line.header.frame_id    = mapData.header.frame_id;
	line.header.stamp       = ros::Time(0);
	line.type				= line.LINE_LIST;
	line.action             = line.ADD;
	line.pose.orientation.w = 1.0;
	line.scale.x 			= 0.03;
	line.scale.y			= 0.03;
	line.color.r			= 1.0;   // 0.0/255.0;
	line.color.g			= 0.0;   // 0.0/255.0;
	line.color.b 			= 1.0;   // 236.0/255.0;
	line.color.a 			= 1.0;
	line.lifetime           = ros::Duration();


	// -------------------------------------set the origin of the RRT
	std::vector< std::vector<float>  > V;  
	std::vector<float> xnew; 
	std::cout << ns << "wait for transform"<< std::endl;
	tf::StampedTransform transform;
	int  temp=0;
	while (temp==0){
		try{
			temp=1;
			listener.lookupTransform(mapData.header.frame_id, robot_base_frame, ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
			temp=0;
			ros::Duration(0.1).sleep();
		}
	}

	xnew.push_back(transform.getOrigin().x()+0.3);
	xnew.push_back(transform.getOrigin().y()); 
	V.push_back(xnew);

    // -------------------------------------initialize the previous goal point
    previous_goal_map_frame.header.frame_id = robot_frame;
    previous_goal_map_frame.point.x         = xnew[0]+mapData.info.resolution*3;
    previous_goal_map_frame.point.y         = xnew[1]+mapData.info.resolution*3;
    previous_goal_map_frame.point.z         = 0;

	std:: cout << ns << "RRT Tree starts growing origin: (" << xnew[0] << ", "  << xnew[1] << ")" << std::endl;

    // -------------------------------------clear clicked points
	points.points.clear();
	// pub_frontier.publish(points);
    //---------------------------------------------------------------
    //------------------     Main   Loop     ------------------------
    //---------------------------------------------------------------
    // int number_iteration_nofrontier = 0;
    int number_iteration = 0;
	while (ros::ok()){

        // ------------------------------- find fronters
        map_lengthX = mapData.info.width*mapData.info.resolution;
        map_lengthY = mapData.info.height*mapData.info.resolution;
        // std::cout << "start rrt tree growing" << std::endl;
        // Sample free
        x_rand.clear();
        xr=(drand()*map_lengthX) + mapData.info.origin.position.x;
        yr=(drand()*map_lengthY) + mapData.info.origin.position.y;
        x_rand.push_back( xr ); x_rand.push_back( yr );
        
        // ------------------ Nearest
        x_nearest=Nearest(V,x_rand);

        // ------------------ Steer
        x_new=Steer(x_nearest, x_rand, eta);

        // ------------------ ObstacleFree -1:unkown (frontier region)  0:free 1:obstacle
        char checking=ObstacleFree(x_nearest, x_new, costmapData);

        if (checking=='2'){ // '2' means frontier
            exploration_goal.header.stamp=ros::Time(0);
            exploration_goal.header.frame_id=mapData.header.frame_id;
            exploration_goal.point.x=x_new[0];
            exploration_goal.point.y=x_new[1];
            exploration_goal.point.z=0.0;
            // if(mapValue(mapData, exploration_goal)==-1 || mapValue(costmapData, exploration_goal) < costmap_pixel_threshold){
            frontiers.push_back(exploration_goal);
            // }
            // points.points.push_back(exploration_goal.point);
            line.points.push_back(exploration_goal.point);
            p.x=x_nearest[0]; 
            p.y=x_nearest[1]; 
            p.z=0.0;
            line.points.push_back(p);
            // points.pose.
        }
        else if (checking=='0'){ // 0 means free
            V.push_back(x_new);
            
            p.x=x_new[0]; 
            p.y=x_new[1]; 
            p.z=0.0;
            line.points.push_back(p);
            p.x=x_nearest[0]; 
            p.y=x_nearest[1]; 
            p.z=0.0;
            line.points.push_back(p);	
            
        }
        shapes_pub.publish(line); 
        number_iteration++; 

        // ------------------------------- add previous goal
        if(mapValue(costmapData, previous_goal_map_frame)!=-1 || mapValue(costmapData, previous_goal_map_frame) > costmap_pixel_threshold){
        }
        else{
            frontiers.push_back(previous_goal_map_frame);
        }

        // ------------------------------- remove old frontiers whose places have already been explored & the invalid frontiers.
        // std::cout <<  ns << "detects frontiers' number " << frontiers.size() << std::endl;
        for(int i= frontiers.size()-1; i>-1;i--){
            if(mapValue(costmapData, frontiers[i])!=-1 || mapValue(costmapData, frontiers[i]) > costmap_pixel_threshold){
                frontiers.erase(frontiers.begin() + i);
            }
        }

        // // if(frontiers.size() == 0){number_iteration_nofrontier++;}
        // rate.sleep();
        // continue;
        if( ((frontiers.size() - detectFrontierNum) > 25) || (number_iteration > 500) ){

            // ------------------------------- clear detection's iteration recording.
            number_iteration = 0;  

            // ------------------------------- initializes current goal's information.
            double current_goal_score = -500.0, current_goal_infoGain = -500.0;
            int current_goal_idx   = -1;
            
         
            // ------------------------------- remove frontiers lying out of the valid map (uncertainty of map will cause some frontiers lie out of valid map).
            for(int i= frontiers.size()-1; i>-1;i--){
                bool condition_ = false;
                double x_,y_;
                int tempResult[4];
                x_ = frontiers[i].point.x;
                y_ = frontiers[i].point.y;
                int idx = (floor((y_-mapData.info.origin.position.y)/mapData.info.resolution)*mapData.info.width)+(floor((x_+mapData.info.resolution-mapData.info.origin.position.x)/mapData.info.resolution));
                tempResult[0] = costmapData.data[idx];
                idx = floor((y_+mapData.info.resolution-mapData.info.origin.position.y)/mapData.info.resolution)*mapData.info.width+floor((x_-mapData.info.origin.position.x)/mapData.info.resolution);
                tempResult[1] = costmapData.data[idx];
                idx = (floor((y_-mapData.info.origin.position.y)/mapData.info.resolution)*mapData.info.width)+(floor((x_-mapData.info.resolution-mapData.info.origin.position.x)/mapData.info.resolution));
                tempResult[2] = costmapData.data[idx];
                idx = (floor((y_-mapData.info.resolution-mapData.info.origin.position.y)/mapData.info.resolution)*mapData.info.width)+(floor((x_-mapData.info.origin.position.x)/mapData.info.resolution));
                tempResult[3] = costmapData.data[idx];
                // if surring pixels are all unknown, elminate it.
                if( (tempResult[0]+tempResult[1]+tempResult[2]+tempResult[3]) == -4){
                    frontiers.erase(frontiers.begin() + i);
                }
            }
            detectFrontierNum = frontiers.size();

            // ------------------------------- start a new iteration if no frontiers remain.          
            // std::cout <<  ns << "remains frontiers' number " << frontiers.size() << std::endl;
            if(frontiers.size() == 0){
                rate.sleep();
                ros::spinOnce();
                continue;
            }
                        
            // ------------------------------- display valid frontiers.
            points.points.clear();
            for(int i= frontiers.size()-1; i>-1;i--){
                points.points.push_back(frontiers[i].point);
            }
            pub_frontier.publish(points);

            // ------------------------------- find the goal which has the highest score.
            try{
                listener.lookupTransform(mapData.header.frame_id, robot_base_frame, ros::Time(0), transform);
            }
            catch (tf::TransformException ex){
                std::cout << "don't receive transform from robot frame to map's frame" << std::endl;
                continue;
            }

            for(int i=0; i<frontiers.size(); i++){
                double score, pixel, travel_distance, infoGain;
                infoGain = informationRectangleGain(costmapData, frontiers[i], info_radius);
                // if(infoGain<0.2 || pixel > costmap_pixel_threshold){continue;}
                travel_distance =  norm2Dpoints(transform.getOrigin().x(), transform.getOrigin().y(), frontiers[i]);
                // hysteresis here is to reduce overlap as much as possible
                if(travel_distance <= hysteresis_radius){infoGain*=hysteresis_gain;}
                score = 3*infoGain - travel_distance;
                // std::cout << ns <<  "score: "<< score << std::endl;
                if(score > current_goal_score){
                    current_goal_score    = score;
                    current_goal_infoGain = infoGain;
                    current_goal_idx      = i;
                }
            }

            if(current_goal_idx == -1){
                continue;
            }
            // std::cout << ns <<  "Goal infoGain new: "<< current_goal_infoGain  << std::endl;
            // std::cout << ns <<  "Goal index: "<< current_goal_idx << std::endl;
                
            if(frontiers[current_goal_idx].header.frame_id != robot_frame){
                try{
                    // std::cout << "try converts wait for transform" << std::endl;
                    if(listener.waitForTransform(frontiers[current_goal_idx].header.frame_id, robot_frame, ros::Time(0), ros::Duration(5))){
                        // std::cout << "try converts goal point" << std::endl;
                        listener.transformPoint(robot_frame, frontiers[current_goal_idx], goalRobotFrame);
                        // std::cout << "convert done" << std::endl;
                    }
                }
                catch (tf::TransformException ex){
                    std::cout << "don't receive transform from robot frame to frontier's frame" << std::endl;
                    continue;
                }
            }
            else{
                goalRobotFrame = frontiers[current_goal_idx];
            }


            // Send goal to dmce simulation environment
            // std::cout << "Sending goal..." << std::endl;
            dmce_msgs::RobotPlan dmce_plan;
            dmce_plan.robotId = this_robot_idx;
            dmce::pos_t targetPos;
            targetPos.x() = frontiers[current_goal_idx].point.x;
            targetPos.y() = frontiers[current_goal_idx].point.y;
            dmce_plan.path.poses.push_back(dmce::posToPose(targetPos));
            dmce_goalPublisher.publish(dmce_plan);


            // std::cout << ns <<  "flush frontiers" << std::endl;
            // frontiers.clear();
            // std::vector<geometry_msgs::PointStamped>().swap(frontiers);
        }

        if(n_robot>1){
            bool reset_rrt = false;
            for (int i_ = 0; i_ < n_robot; i_++){
                if(i_ + 1 == this_robot_idx){
                    continue;
                }

                tf::StampedTransform transform_;
                try{
                    listener.lookupTransform(robot_frame, robots_frame[i_], ros::Time(0), transform_);
                }
                catch (tf::TransformException &ex) {
                    // ROS_ERROR("[%s] Failed to get transform %s!", ns.c_str(), robots_frame[i_].c_str());
                    continue;
                }

                float dis_origin = 0;
                dis_origin = abs(transform_.getOrigin().x() - transform_vec[i_].getOrigin().x()) ;
                dis_origin += abs(transform_.getOrigin().y() - transform_vec[i_].getOrigin().y()) ;
                dis_origin += abs(transform_.getOrigin().z() - transform_vec[i_].getOrigin().z()) ;
                dis_origin += abs(transform_.getRotation().x() - transform_vec[i_].getRotation().x()) ;
                dis_origin += abs(transform_.getRotation().y() - transform_vec[i_].getRotation().y()) ;
                dis_origin += abs(transform_.getRotation().z() - transform_vec[i_].getRotation().z()) ;
                dis_origin += abs(transform_.getRotation().w() - transform_vec[i_].getRotation().w()) ;

                if(dis_origin > THRESHOLD_TRANSFORM){
                    reset_rrt = true;
                    transform_vec[i_].setOrigin(transform_.getOrigin());
                    transform_vec[i_].setRotation(transform_.getRotation());
                }
            }

            if(reset_rrt){
                std::cout << ns <<  "flush frontiers" << std::endl;
                frontiers.clear();
                std::vector<geometry_msgs::PointStamped>().swap(frontiers);

                line.points.clear();
                std::vector<std::vector<float> >().swap(V); 
                tf::StampedTransform transform_;
                int  temp_=0;
                std::vector<float> xnew_;
                while (temp_==0){
                    try{
                        listener.lookupTransform(mapData.header.frame_id, robot_base_frame, ros::Time(0), transform_);
                        temp_=1;
                    }
                    catch (tf::TransformException ex){
                        ros::Duration(0.1).sleep();
                    }
                }

                xnew_.push_back(transform_.getOrigin().x());
                xnew_.push_back(transform_.getOrigin().y()); 
                V.push_back(xnew_);
            }
        }

		ros::spinOnce();
		rate.sleep();
        // std::cout << ns << "loop ends" << std::endl;
	}

	return 0;
}
