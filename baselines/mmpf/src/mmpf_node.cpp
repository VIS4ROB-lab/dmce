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
*******************************************************************************/

// --------------------- ros things
#include "ros/ros.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_listener.h>
#include "nav_msgs/OccupancyGrid.h"
#include <actionlib/client/simple_action_client.h>
// #include "cartographer_ros_msgs/TrajectoryQuery.h"
#include <mutex>
#include "std_msgs/String.h"

// Headers from dmce simulation
#include "dmce_core/TypeDefs.hpp"
#include "dmce_msgs/RobotPlan.h"


// --------------------- MMPF include things
#include<fstream>
#include<sstream>
#include<iostream>
#include<iomanip>
#include<string>
#include<cstdlib>
#define RESOLUTION 0.05
#define K_ATTRACT 1
#define ETA_REPLUSIVE 3
#define DIS_OBTSTACLE 6
#define DISTANCE_THRES_VALID_OBSTACLE 160
#define THRESHOLD_TRANSFORM 0.5
#define ROBOT_INTERFERE_RADIUS 50
#define LARGEST_MAP_DISTANCE 500000 // 500*1000

// #define FILE_DEBUG
// --------------------- ROS global variable
nav_msgs::OccupancyGrid     mapData, costmapData;
std::mutex _mt;
geometry_msgs::PointStamped clickedpoint;
visualization_msgs::Marker  points,line;

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

void dismapConstruction_start_target(std::vector<int>& dismap_, const std::vector<int>& dismap_backup_, const std::pair<int, int>& curr, int HEIGHT, int WIDTH){
    std::vector<std::pair<int, int>> curr_iter;
    std::vector<std::pair<int, int>> next_iter;

    // initialize the dis_map with number of dismapBackup
    //memcpy(dismap_, dismap_backup_, sizeof(int) * HEIGHT * WIDTH);
    for (size_t i = 0; i < dismap_backup_.size(); ++i) {
      dismap_[i] = dismap_backup_[i];
    }

    int iter = 1;
    curr_iter.push_back({curr.first, curr.second});

    // change pixel of the starting location to -500 to avoid being changed. After processing dismap, we changed it back to 0;
    dismap_[(curr.first) * WIDTH + curr.second] = -500;

    while (curr_iter.size() > 0) {
        if(iter>LARGEST_MAP_DISTANCE){
            std::cout << "distance exceeds MAXIMUM SETUP" << std::endl;
            return;
        }
        for (int i = 0; i < curr_iter.size(); i++) {
            if ((curr_iter[i].first + 1) * WIDTH + curr_iter[i].second < dismap_.size() &&
                dismap_[(curr_iter[i].first + 1) * WIDTH + curr_iter[i].second] == 0) {
                dismap_[(curr_iter[i].first + 1) * WIDTH + curr_iter[i].second] = iter;
                next_iter.push_back({curr_iter[i].first + 1, curr_iter[i].second});
            }

            if ((curr_iter[i].first) * WIDTH + curr_iter[i].second + 1 < dismap_.size() &&
                dismap_[(curr_iter[i].first) * WIDTH + curr_iter[i].second + 1] == 0) {
                dismap_[(curr_iter[i].first) * WIDTH + curr_iter[i].second + 1] = iter;
                next_iter.push_back({curr_iter[i].first, curr_iter[i].second + 1});
            }

            if ((curr_iter[i].first - 1) * WIDTH + curr_iter[i].second < dismap_.size() &&
                dismap_[(curr_iter[i].first - 1) * WIDTH + curr_iter[i].second] == 0) {
                dismap_[(curr_iter[i].first - 1) * WIDTH + curr_iter[i].second] = iter;
                next_iter.push_back({curr_iter[i].first - 1, curr_iter[i].second});
            }

            if ((curr_iter[i].first) * WIDTH + curr_iter[i].second - 1 < dismap_.size() &&
                dismap_[(curr_iter[i].first) * WIDTH + curr_iter[i].second - 1] == 0) {
                dismap_[(curr_iter[i].first) * WIDTH + curr_iter[i].second - 1] = iter;
                next_iter.push_back({curr_iter[i].first, curr_iter[i].second - 1});
            }
        }
        //for (int curr_idx = 0; curr_idx < curr_iter.size(); curr_idx++){
        //    delete [] curr_iter[curr_idx];
        //}
        curr_iter.swap(next_iter);
        std::vector<std::pair<int, int>>().swap(next_iter);
        iter++;
    }
    dismap_[(curr.first) * WIDTH + curr.second] = 0;  // int only zero is available
    return ;
}

// --------------------------------------------------//
// ----------------------Main------------------------//
// --------------------------------------------------//
int main(int argc, char** argv) {
    // ---------------------------------------- ros initialization;
    std::string map_topic, costmap_topic, trajectory_query_name, output_file, output_map_file;
    std::string robot_frame, robot_base_frame;
    int rateHz;
    ros::init(argc, argv, "mmpf_node");
    ros::NodeHandle nh;
    std::string nodename, ns, robot_ano_frame_suffix, robot_ano_frame_preffix;
    int rotation_count   = 0, n_robot, this_robot_idx;
    float inflation_radius;    // max 4 degree;
    bool start_condition = true;
    int no_targets_count = 0;

    float rotation_w[3]  = {0.866,  0.500, 1.0};
    float rotation_z[3]  = {0.5  , -0.866, 0.0};

    nodename=ros::this_node::getName();

    ros::param::param<std::string>(nodename+"/map_topic", map_topic, "robot1/map");
    ros::param::param<std::string>(nodename+"/costmap_topic", costmap_topic, "robot1/move_base/global_costmap/costmap");
    ros::param::param<std::string>(nodename+"/robot_base_frame", robot_base_frame, "robot1/base_footprint");
    ros::param::param<std::string>(nodename+"/robot_frame", robot_frame, "robot1/map");
    ros::param::param<std::string>(nodename+"/namespace", ns, "robot1");
    ros::param::param<int>(nodename+"/rate", rateHz, 1);
    // ros::param::param<float>(nodename+"/inflation_radius", inflation_radius, 2);
    std::string diameterParam = "/robot/diameter";
    if (!ros::param::has(diameterParam)) {
        ROS_ERROR("Parameter %s not found!", diameterParam.c_str());
        throw std::runtime_error("Required parameter not found on ROS parameter server!");
    }
    ros::param::get(diameterParam, inflation_radius);
    inflation_radius = 4*inflation_radius;
    ros::param::param<int>(nodename+"/n_robot", n_robot, 1);
    ros::param::param<int>(nodename+"/this_robot_idx", this_robot_idx, 1);
    ros::param::param<std::string>(nodename+"/robot_ano_frame_preffix", robot_ano_frame_preffix, "robot");
    ros::param::param<std::string>(nodename+"/robot_ano_frame_suffix", robot_ano_frame_suffix, "/base_footprint");
    ros::param::param<std::string>(nodename+"/trajectory_query_name", trajectory_query_name, "robot1/trajectory_query");
    ros::param::param<std::string>(nodename+"/output_file", output_file, "/home/nics/work/SMMR-Explore/src/mmpf/robot1_mmpf_trajectory.txt");
    ros::param::param<std::string>(nodename+"/output_map_file", output_map_file, "/home/nics/work/SMMR-Explore/src/mmpf/robot1_mmpf_explored_map.txt");

    ros::Rate rate(rateHz);

    // ------------------------------------- subscribe the map topics & clicked points
    ros::Subscriber sub       = nh.subscribe<nav_msgs::OccupancyGrid>(map_topic, 20 ,mapCallBack);
    ros::Subscriber costMapSub= nh.subscribe<nav_msgs::OccupancyGrid>(costmap_topic, 20, costmapMergedCallBack);

    // ------------------------------------- subscribe the map topics & clicked points
    tf::TransformListener listener;

    // ------------------------------------- publish the detected points for following processing & display
    ros::Publisher pub_shapes   = nh.advertise<visualization_msgs::Marker>(nodename+"_shapes", 100);
    ros::Publisher pub_centroid = nh.advertise<visualization_msgs::Marker>(nodename+"_detected_frontier_centroid", 10);

    // Publisher for sending plan to dmce simulation
    ros::Publisher dmce_goalPublisher = nh.advertise<dmce_msgs::RobotPlan>("mmpf_plan", 10);


    // ------------------------------------- wait until map is received
    std::cout << ns << "wait for map "<< std::endl;
    while ( mapData.header.seq < 1  or  mapData.data.size() < 1 ){  ros::spinOnce();  ros::Duration(0.1).sleep(); }
    std::cout << ns << "wait for costmap "<< std::endl;
    while ( costmapData.data.size()<1)  {  ros::spinOnce();  ros::Duration(0.1).sleep();}

    move_base_msgs::MoveBaseGoal robotGoal;
    robotGoal.target_pose.header.frame_id = robot_frame;
    robotGoal.target_pose.pose.position.z = 0;
    robotGoal.target_pose.pose.orientation.z = 1.0;

    // ------------------------------------- initilize the visualized points & lines
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

    // -------------------------------------Initialize all robots' frame;
    std::string robots_frame[n_robot];

    for (int i = 1; i < n_robot+1; i++){

        std::stringstream ss;
        ss << robot_ano_frame_preffix;
        ss << i;
        ss << robot_ano_frame_suffix;

        robots_frame[i-1] = ss.str();
    }

    // -------------------------------------clear clicked points
    points.points.clear();
    pub_shapes.publish(points);

    while(ros::ok()){

        // ---------------------------------------- variables from ROS input;
        int HEIGHT = mapData.info.height;
        int WIDTH  = mapData.info.width;

        // ---------------------------------------- define variables;
        //std::vector<int* > obstacles, path, targets;
        std::vector<std::pair<int, int>> obstacles, path, targets;
        int currentLoc[2], goal[2]; //target[2], obstacle[2]
        float  minDis2Frontier;
        std::ifstream infile;
        int map[HEIGHT*WIDTH];
        //std::vector<int * > dismap_targets_ptr;
        //int* dismap_backup = new int[HEIGHT*WIDTH];
        std::vector<int> dismap_backup(HEIGHT*WIDTH);

        // ---------------------------------------- initialize the map & dismap
        for (int i=0; i<HEIGHT; i++)
        {
            for (int j=0; j<WIDTH; j++)
            {
                map[i*WIDTH + j] = (int) costmapData.data[i*costmapData.info.width + j];
                dismap_backup[i*WIDTH + j] = map[i*WIDTH + j];
            }
        }

        // ------------------------------------------ find the obstacles & targets
        for (int i = 2; i < HEIGHT-2; i++){
            for (int j = 2; j < WIDTH-2; j++){
                if(map[i*WIDTH + j] == 100){
                    //obstacles.push_back(new int[2]{i,j});
                    obstacles.push_back({i,j});
                }
                else if(map[i*WIDTH + j] == -1){
                    // accessiable frontiers
                    int numFree = 0, temp1 = 0;

                    if (map[(i + 1)*WIDTH + j] == 0){
                        temp1 += (map[(i + 2)*WIDTH + j    ] == 0) ? 1 : 0;
                        temp1 += (map[(i + 1)*WIDTH + j + 1] == 0) ? 1 : 0;
                        temp1 += (map[(i + 1)*WIDTH + j - 1] == 0) ? 1 : 0;
                        numFree += (temp1 > 0);
                    }

                    if (map[i*WIDTH + j + 1] == 0){
                        temp1 = 0;
                        temp1 += (map[      i*WIDTH + j + 2] == 0) ? 1 : 0;
                        temp1 += (map[(i + 1)*WIDTH + j + 1] == 0) ? 1 : 0;
                        temp1 += (map[(i - 1)*WIDTH + j + 1] == 0) ? 1 : 0;
                        numFree += (temp1 > 0);
                    }

                    if (map[(i - 1) *WIDTH + j] == 0){
                        temp1 = 0;
                        temp1 += (map[(i - 1)*WIDTH + j + 1] == 0) ? 1 : 0;
                        temp1 += (map[(i - 1)*WIDTH + j - 1] == 0) ? 1 : 0;
                        temp1 += (map[(i - 2)*WIDTH + j    ] == 0) ? 1 : 0;
                        numFree += (temp1 > 0);
                    }

                    if (map[i * WIDTH + j - 1] == 0){
                        temp1 = 0;
                        temp1 += (map[    i  *WIDTH + j - 2] == 0) ? 1 : 0;
                        temp1 += (map[(i + 1)*WIDTH + j - 1] == 0) ? 1 : 0;
                        temp1 += (map[(i - 1)*WIDTH + j - 1] == 0) ? 1 : 0;
                        numFree += (temp1 > 0);
                    }

                    if( numFree > 0 ) {
                        //targets.push_back(new int[2]{i,j});
                      targets.push_back({i,j});
                    }
                }
            }
        }

         // ------------------------------------------ remove targets within the inflation layer of costmap.
        {
            for (int idx_target = targets.size()-1; idx_target >= 0; idx_target--) {

                float loc_x = targets[idx_target].second*mapData.info.resolution + mapData.info.origin.position.x;
                float loc_y = targets[idx_target].first*mapData.info.resolution + mapData.info.origin.position.y;
                int index_costmap = (loc_y - costmapData.info.origin.position.y)/costmapData.info.resolution * costmapData.info.width + (loc_x - costmapData.info.origin.position.x)/costmapData.info.resolution;
                if (costmapData.data[index_costmap] >0){
                    targets.erase(targets.begin() + idx_target);
                    continue;
                }
            }
            // std::cout << "(costmap) number targets after erase" << targets.size() << std::endl;
        }

        // ------------------------------------------ remove targets within the inflation radius of obstacles.
        {
            for(int idx_target = targets.size()-1; idx_target>=0; idx_target--) {
                for (int i = 0; i < obstacles.size(); i++) {
                    if (abs(targets[idx_target].first - obstacles[i].first) +
                        abs(targets[idx_target].second - obstacles[i].second) < inflation_radius) {
                        targets.erase(targets.begin() + idx_target);
                        break;
                    }
                }
            }
        }

        // ------------------------------------------ exploration finish detection
        if(targets.size() == 0){
            no_targets_count ++;
            std::cout << ns << "no targets count = " << no_targets_count << std::endl;
            ros::spinOnce();
            rate.sleep();
            continue;
        }
        else{
            no_targets_count = 0;
        }


        // ---------------------------------------- define the current point;
        tf::StampedTransform  transform;
        int  temp=0;
        while (temp==0){
            try{
                temp=1;
                listener.lookupTransform( mapData.header.frame_id, robot_base_frame, ros::Time(0), transform );
            }
            catch( tf::TransformException ex ){
                temp=0;
                ros::Duration(0.1).sleep();
            }
        }
        currentLoc[0] = floor((transform.getOrigin().y()-mapData.info.origin.position.y)/mapData.info.resolution);
        currentLoc[1] = floor((transform.getOrigin().x()-mapData.info.origin.position.x)/mapData.info.resolution);
        path.push_back( {currentLoc[0], currentLoc[1]} );
        //path.push_back(currentLoc);

        for(int i = 0; i< obstacles.size(); i++){
            dismap_backup[(obstacles[i].first)*WIDTH + obstacles[i].second] = -2;
        }

        // ------------------------------------------ cluster targets into different groups and find the center of each group.
        // Note: x & y value of detected targets are in increasing order because of the detection is in laser scan order.
        //std::vector<int* > target_process(targets);
        std::vector<std::pair<int, int>> target_process(targets);
        std::vector<std::pair<int, int> > cluster_center;
        std::vector<int>   infoGain_cluster;

        while(target_process.size() > 0){
            //std::vector<int* > target_cluster;
            std::vector<std::pair<int, int> > target_cluster;
            target_cluster.push_back(target_process.back());
            target_process.pop_back();

            bool condition = true;
            while(condition){
                condition = false;
                int size_target_process = target_process.size();
                for (int i = size_target_process-1; i >= 0 ; i--){
                    for (int j = 0; j < target_cluster.size(); j++){
                        int dis_ = abs(target_process[i].first - target_cluster[j].first) +  abs(target_process[i].second - target_cluster[j].second);
                        if(dis_ < 3){
                            target_cluster.push_back(target_process[i]);
                            target_process.erase(target_process.begin() + i);
                            condition = true;
                            break;
                        }
                    }
                }
            }

            int center_[2]={0, 0};
            int num_ = target_cluster.size();
            for(int i = 0; i < num_; i++){
                center_[0] += target_cluster[i].first;
                center_[1] += target_cluster[i].second;
            }

            float center_float[2] = {float(center_[0]), float(center_[1])};
            center_float[0] = center_float[0]/float(num_);
            center_float[1] = center_float[1]/float(num_);

            float min_dis_ = 100.0;
            int min_idx_   = 10000;
            for(int i = 0; i < num_; i++){
                float temp_dis_ = abs(center_float[0]-float(target_cluster[i].first)) + abs(center_float[1]-float(target_cluster[i].second));
                if(temp_dis_ < min_dis_){
                    min_dis_ = temp_dis_;
                    min_idx_ = i;
                }
            }

            cluster_center.push_back({target_cluster[min_idx_].first, target_cluster[min_idx_].second});
            infoGain_cluster.push_back(num_);
        }

        // ------------------------------------------ Display Cluster centroids
        points.points.clear();
        for(int i = 0; i < cluster_center.size(); i++){
            geometry_msgs::Point temp;
            temp.x = cluster_center[i].second * mapData.info.resolution + mapData.info.origin.position.x;
            temp.y = cluster_center[i].first * mapData.info.resolution + mapData.info.origin.position.y;
            temp.z = 0;
            points.points.push_back(temp);
        }
        pub_centroid.publish(points);

        // ------------------------------------------ Generate Dismap starting from targets
        int cluster_num = cluster_center.size();
        //int** dismap_target = new int* [cluster_num];
        std::vector<std::vector<int>> dismap_target(cluster_num, std::vector<int>());
        for (int i = 0; i<cluster_num; i++){
            //dismap_target[i] = new int[HEIGHT*WIDTH];
          dismap_target[i].resize(HEIGHT * WIDTH);
        }

        for(int i = 0; i< cluster_num; i++) {
            dismapConstruction_start_target(dismap_target[i], dismap_backup, cluster_center[i], HEIGHT, WIDTH);
            //dismap_targets_ptr.push_back(dismap_target[i]);
        }

        // ------------------------------------------ receive robots' locations
        bool ifmapmerged_vec[n_robot];
        tf::StampedTransform transform_robot[n_robot];

        for(int i = 0; i < n_robot; i++){

            ifmapmerged_vec[i] = false;
            if(i == this_robot_idx-1){
                continue;
            }

            tf::StampedTransform transform_;
            try{
                listener.lookupTransform(robot_frame, robots_frame[i], ros::Time(0), transform_);
            }
            catch (tf::TransformException &ex){
                ROS_ERROR("[%s] Failed to get transform %s!", ns.c_str(), robots_frame[i].c_str());
                continue;
            }

            // std::cout<< robot_frame << " receive " << robots_frame[i] << " (" << transform_.getOrigin().getX() << ", "<<   transform_.getOrigin().getY()<< " )" << std::endl;
            transform_robot[i] = transform_;
            ifmapmerged_vec[i] = true;
            // std::cout<< robot_frame << " receive " << robots_frame[i] << " (" << transform_robot[i].getOrigin().getX() << ", "<<   transform_robot[i].getOrigin().getY()<< " )" << std::endl;

        }

        // ------------------------------------------ calculate path.
        int iteration = 1;
        int currentPotential = 10000;
        int riverFlowPotentialGain = 1;
        minDis2Frontier  = 10000;  // a random initialized value greater than all possible distances.
        while(iteration < 3000 && minDis2Frontier > 1){
            // ------------------------------------------
            // ------------------------------------------
            // ------------------------------------------ get the minimial potential of the points around currentLoc
            {
                // ------------------------------------------ put locations around the current location into loc_around
                float potential[4];
                int min_idx = -1;
                float min_potential = 10000;
                //int* loc_around[4];
                std::vector<std::pair<int, int>> loc_around(4);

                // upper
                //loc_around[0] = new int[2]{currentLoc[0]    , currentLoc[1] + 1};
                loc_around[0] = {currentLoc[0]    , currentLoc[1] + 1};
                // left
                //loc_around[1] = new int[2]{currentLoc[0] - 1, currentLoc[1]};
                loc_around[1] = {currentLoc[0] - 1, currentLoc[1]};
                // down
                //loc_around[2] = new int[2]{currentLoc[0]    , currentLoc[1] - 1};
                loc_around[2] = {currentLoc[0]    , currentLoc[1] - 1};
                // right
                //loc_around[3] = new int[2]{currentLoc[0] + 1, currentLoc[1]};
                loc_around[3] = {currentLoc[0] + 1, currentLoc[1]};

                // ------------------------------------------ calculate potentials of four neighbors of currentLoc
                for (int i = 0; i < 4; i++){
                    int curr_around[2]={loc_around[i].first, loc_around[i].second};

                    { // ------------------------------------ calculate current potential
                        float attract = 0, repulsive = 0;
                        for (int j = 0; j < cluster_center.size(); j++){
                            // int temp_int = dismap_targets_ptr[j][(curr_around[0])*WIDTH + curr_around[1]];
                            //float temp = float(dismap_targets_ptr[j][(curr_around[0])*WIDTH + curr_around[1]]);
                          float temp = float(dismap_target[j][(curr_around[0])*WIDTH + curr_around[1]]);
                            if(temp < 1){
                                continue;
                            }
                            attract     = attract - K_ATTRACT*infoGain_cluster[j]/temp;
                        }

                        // to increase the potential if currend point has been passed before
                        for (int j =0; j < path.size(); j++){
                            if(curr_around[0] == path[j].first && curr_around[1] == path[j].second){
                                attract += riverFlowPotentialGain*5;
                            }
                        }

                        // Add impact of robots.
                        for(int i = 0; i < n_robot; i++){
                            if(ifmapmerged_vec[i] ){
                                int index_[2] = {int(round((transform_robot[i].getOrigin().y() - mapData.info.origin.position.y)/mapData.info.resolution)), int(round((transform_robot[i].getOrigin().x() - mapData.info.origin.position.x)/mapData.info.resolution))};
                                int dis_ = abs(curr_around[0] - index_[0]) + abs(curr_around[1] - index_[1]);
                                if( dis_ < ROBOT_INTERFERE_RADIUS){
                                    int temp_ = (ROBOT_INTERFERE_RADIUS - dis_);
                                    attract += temp_;
                                }
                            }
                        }

                        potential[i] = attract;
                        if(min_potential > potential[i] ){
                            min_potential = potential[i];
                            min_idx = i;
                        }
                    }
                }
                if(currentPotential > min_potential){
                    path.push_back(loc_around[min_idx]);
                    currentPotential = min_potential;

                }
                else{
                    riverFlowPotentialGain++;
                }

                for(int del_idx = 0; del_idx <4 ; del_idx++){
                    if(del_idx != min_idx){
                        //delete [] loc_around[del_idx];
                        loc_around.erase(loc_around.begin() + del_idx);
                    }
                }
            }

            currentLoc[0] = (path.back()).first;
            currentLoc[1] = (path.back()).second;

            for (int i = 0; i < cluster_center.size() ; i++){
                //int temp_dis_ =  dismap_targets_ptr[i][(currentLoc[0])*WIDTH + currentLoc[1]];
              int temp_dis_ =  dismap_target[i][(currentLoc[0])*WIDTH + currentLoc[1]];
                if( (temp_dis_ == 0) && (abs(currentLoc[0]-cluster_center[i].first) + abs(currentLoc[1]-cluster_center[i].second)) > 0){
                    continue;
                }

                if(minDis2Frontier > temp_dis_ ){
                    minDis2Frontier = temp_dis_;
                }
            }
            iteration++;

            // ---------------------------------------- publish path for displaying in rviz
            if(iteration >= 1){
                p.x=(path[path.size()-2]).second * mapData.info.resolution + mapData.info.origin.position.x;
                p.y=(path[path.size()-2]).first * mapData.info.resolution + mapData.info.origin.position.y;
                p.z=0.0;
                line.points.push_back(p);
                p.x=currentLoc[1] * mapData.info.resolution + mapData.info.origin.position.x;
                p.y=currentLoc[0] * mapData.info.resolution + mapData.info.origin.position.y;
                p.z=0.0;
                line.points.push_back(p);
                pub_shapes.publish(line);
            }
        }
        goal[0] = path.back().first;
        goal[1] = path.back().second;

        if(start_condition){
            tf::StampedTransform  transform;
            int  temp=0;
            while (temp==0){
                try{
                    temp=1;
                    listener.lookupTransform( mapData.header.frame_id, robot_base_frame, ros::Time(0), transform );
                }
                catch( tf::TransformException ex ){
                    temp=0;
                    ros::Duration(0.1).sleep();
                }
            }
            int loc_x = transform.getOrigin().x();
            int loc_y = transform.getOrigin().y();

            robotGoal.target_pose.pose.orientation.z = rotation_z[rotation_count];
            robotGoal.target_pose.pose.orientation.w = rotation_w[rotation_count];

            robotGoal.target_pose.pose.position.x = loc_x + 0.2;
            robotGoal.target_pose.pose.position.y = loc_y + 0.2;

            start_condition = false;
        }
        else{
            robotGoal.target_pose.pose.orientation.z = 1;
            robotGoal.target_pose.pose.orientation.w = 0;
            robotGoal.target_pose.pose.position.x = goal[1]*mapData.info.resolution + mapData.info.origin.position.x;
            robotGoal.target_pose.pose.position.y = goal[0]*mapData.info.resolution + mapData.info.origin.position.y;
            robotGoal.target_pose.header.stamp    = ros::Time(0);
            // ac.sendGoal(robotGoal);

            // Send goal to dmce simulation environment
            std::cout << ns << "Sending goal..." << std::endl;
            dmce_msgs::RobotPlan dmce_plan;
            dmce_plan.robotId = this_robot_idx;
            dmce::pos_t targetPos;
            targetPos.x() = robotGoal.target_pose.pose.position.x;
            targetPos.y() = robotGoal.target_pose.pose.position.y;
            dmce_plan.path.poses.push_back(dmce::posToPose(targetPos));
            dmce_goalPublisher.publish(dmce_plan);
        }
        line.points.clear();

        // ------------------------------------------- clear memory
//        delete [] dismap_backup;
//        for (int i = 0; i<cluster_num; i++){
//            delete []  dismap_target[i];
//        }
//        delete [] dismap_target;

        // ------------------------------------------- keep frequency stable
        // _mt.unlock();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;

}
