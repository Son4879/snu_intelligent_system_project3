//state definition
#define INIT 0
#define RUNNING 1
#define MAPPING 2
#define PATH_PLANNING 3
#define FINISH -1

#include <unistd.h>
#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <project2/rrtTree.h>
#include <tf/transform_datatypes.h>
#include <project2/pid.h>
#include <math.h>
#include <pwd.h>
#include <iostream>

//map spec
cv::Mat map;
double res;
int map_y_range;
int map_x_range;
double map_origin_x;
double map_origin_y;
double world_x_min;
double world_x_max;
double world_y_min;
double world_y_max;

//parameters we should adjust : K, margin, MaxStep
int margin = 8;
int K = 10000;
double MaxStep = 1.2;

//way points
std::vector<point> waypoints;

//path
std::vector<traj> path_RRT;

std::vector<int> path_end_idx;

//robot
point robot_pose;
ackermann_msgs::AckermannDriveStamped cmd;
gazebo_msgs::ModelStatesConstPtr model_states;

//FSM state
int state;

//function definition
void set_waypoints();
void set_angle();
void generate_path_RRT();
void callback_state(gazebo_msgs::ModelStatesConstPtr msgs);
void setcmdvel(double v, double w);

int main(int argc, char** argv){
    ros::init(argc, argv, "rrt_main");
    ros::NodeHandle n;

    // Initialize topics
    ros::Subscriber gazebo_pose_sub = n.subscribe("/gazebo/model_states",100,callback_state);
    ros::Publisher cmd_vel_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/low_level/ackermann_cmd_mux/output",100);
    ros::ServiceClient gazebo_spawn = n.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
    ros::ServiceClient gazebo_set = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    printf("Initialize topics\n");

    // Load Map

    char* user = getpwuid(getuid())->pw_name;
    map = cv::imread((std::string("/home/") + std::string(user) +
                      std::string("/catkin_ws/src/project3/src/ground_truth_map.pgm")).c_str(), CV_LOAD_IMAGE_GRAYSCALE);

    map_y_range = map.cols;
    map_x_range = map.rows;
    map_origin_x = map_x_range/2.0 - 0.5;
    map_origin_y = map_y_range/2.0 - 0.5;
    world_x_min = -4.5;
    world_x_max = 4.5;
    world_y_min = -13.5;
    world_y_max = 13.5;
    res = 0.05;
    printf("Load map\n");


    if(!map.data)                              // Check for invalid input
    {
        printf("Could not open or find the image\n");
        return -1;
    }

    // Set Way Points
    set_waypoints();
    printf("Set way points\n");

    set_angle();
//    printf("set_angle called \n");
    // RRT
    generate_path_RRT();
    printf("Generate RRT\n");
printf("path size : %d\n", path_RRT.size());

    // FSM
    state = INIT;
//    state = FINISH;
    bool running = true;
    int look_ahead_idx;
    ros::Rate control_rate(60);
// define
    int rrt_next = 1;
    PID pid_ctrl;
    double max_turn = 0.6;

    while(running){
        switch (state) {
            case INIT: {
                look_ahead_idx = 0;
	            printf("path size : %d\n", path_RRT.size());
                //visualize path
	            ros::spinOnce();
                for(int i = 0; i < path_RRT.size(); i++){
		            for(int j = 0; j < model_states->name.size(); j++){
                        std::ostringstream ball_name;
                        ball_name << i;
                	    if(std::strcmp(model_states->name[j].c_str(), ball_name.str().c_str()) == 0){
                            //initialize robot position
                            geometry_msgs::Pose model_pose;
                            model_pose.position.x = path_RRT[i].x;
                            model_pose.position.y = path_RRT[i].y;
                            model_pose.position.z = 0.7;
                            model_pose.orientation.x = 0.0;
                            model_pose.orientation.y = 0.0;
                            model_pose.orientation.z = 0.0;
                            model_pose.orientation.w = 1.0;

                            geometry_msgs::Twist model_twist;
                            model_twist.linear.x = 0.0;
                            model_twist.linear.y = 0.0;
                            model_twist.linear.z = 0.0;
                            model_twist.angular.x = 0.0;
                            model_twist.angular.y = 0.0;
                            model_twist.angular.z = 0.0;

                            gazebo_msgs::ModelState modelstate;
                            modelstate.model_name = ball_name.str();
                            modelstate.reference_frame = "world";
                            modelstate.pose = model_pose;
                            modelstate.twist = model_twist;

                            gazebo_msgs::SetModelState setmodelstate;
                            setmodelstate.request.model_state = modelstate;

                            gazebo_set.call(setmodelstate);
                            continue;
                        }
        		    }
	        
                    gazebo_msgs::SpawnModel model;
                    model.request.model_xml = std::string("<robot name=\"simple_ball\">") +
			        std::string("<static>true</static>") +
                            std::string("<link name=\"ball\">") +
                            std::string("<inertial>") +
                            std::string("<mass value=\"1.0\" />") +
                            std::string("<origin xyz=\"0 0 0\" />") +
                            std::string("<inertia  ixx=\"1.0\" ixy=\"1.0\"  ixz=\"1.0\"  iyy=\"1.0\"  iyz=\"1.0\"  izz=\"1.0\" />") +
                            std::string("</inertial>") +
                            std::string("<visual>") +
                            std::string("<origin xyz=\"0 0 0\" rpy=\"0 0 0\" />") +
                            std::string("<geometry>") +
                            std::string("<sphere radius=\"0.09\"/>") +
                            std::string("</geometry>") +
                            std::string("</visual>") +
                            std::string("<collision>") +
                            std::string("<origin xyz=\"0 0 0\" rpy=\"0 0 0\" />") +
                            std::string("<geometry>") +
                            std::string("<sphere radius=\"0.09\"/>") +
                            std::string("</geometry>") +
                            std::string("</collision>") +
                            std::string("</link>") +
                            std::string("<gazebo reference=\"ball\">") +
                            std::string("<mu1>10</mu1>") +
                            std::string("<mu2>10</mu2>") +
                            std::string("<material>Gazebo/Blue</material>") +
                            std::string("<turnGravityOff>true</turnGravityOff>") +
                            std::string("</gazebo>") +
                            std::string("</robot>");

                    std::ostringstream ball_name;
                    ball_name << i;
                    model.request.model_name = ball_name.str();
                    model.request.reference_frame = "world";
                    model.request.initial_pose.position.x = path_RRT[i].x;
                    model.request.initial_pose.position.y = path_RRT[i].y;
                    model.request.initial_pose.position.z = 0.7;
                    model.request.initial_pose.orientation.w = 0.0;
                    model.request.initial_pose.orientation.x = 0.0;
                    model.request.initial_pose.orientation.y = 0.0;
                    model.request.initial_pose.orientation.z = 0.0;
                    gazebo_spawn.call(model);
                    ros::spinOnce();
                }
                printf("Spawn path\n");
	
                //initialize robot position
                geometry_msgs::Pose model_pose;
                model_pose.position.x = waypoints[0].x;
                model_pose.position.y = waypoints[0].y;
                model_pose.position.z = 0.3;
                model_pose.orientation.x = 0.0;
                model_pose.orientation.y = 0.0;
                model_pose.orientation.z = 0.0;
                model_pose.orientation.w = 1.0;

                geometry_msgs::Twist model_twist;
                model_twist.linear.x = 0.0;
                model_twist.linear.y = 0.0;
                model_twist.linear.z = 0.0;
                model_twist.angular.x = 0.0;
                model_twist.angular.y = 0.0;
                model_twist.angular.z = 0.0;

                gazebo_msgs::ModelState modelstate;
                modelstate.model_name = "racecar";
                modelstate.reference_frame = "world";
                modelstate.pose = model_pose;
                modelstate.twist = model_twist;

                gazebo_msgs::SetModelState setmodelstate;
                setmodelstate.request.model_state = modelstate;

                gazebo_set.call(setmodelstate);
                ros::spinOnce();
                ros::Rate(0.33).sleep();

                printf("Initialize ROBOT\n");
                state = RUNNING;
            } break;

            case RUNNING: {
	        //TODO
		while (ros::ok())
                {
                    //current point setting
		    //std::cout << rrt_next << " 2 ";
		
                    point next_point;
                    next_point.x = path_RRT[rrt_next].x;
                    next_point.y = path_RRT[rrt_next].y;
                    next_point.th = path_RRT[rrt_next].th;
                    //ctrl control from robot_position to next_point
                    float control = pid_ctrl.get_control(robot_pose, next_point);
		    float angle = 0;
		    angle += control;
		    if (angle > max_turn) angle = max_turn;
		    if (angle < -max_turn) angle = -max_turn;
                    setcmdvel(1, angle);
                    cmd_vel_pub.publish(cmd);
                    //use robot_pose
                    if (pow(next_point.x - robot_pose.x, 2) + pow(next_point.y - robot_pose.y, 2) <= 0.04) //when reached next path_RRT
                    {
                        printf("robot pose : %.2f,%.2f,%.2f \n", robot_pose.x, robot_pose.y, robot_pose.th);
                        rrt_next++;
                    }
                    if (rrt_next == path_RRT.size()) //when arrived
                    {
                        state = FINISH;
                        break;
                    }
		    ros::spinOnce();
	            control_rate.sleep();
//	            printf("robot pose : %.2f,%.2f,%.2f \n", robot_pose.x, robot_pose.y, robot_pose.th);
//		    std::cout << " " << rrt_next << std::endl;
                }
            } break;

            case FINISH: {
                setcmdvel(0,0);
                cmd_vel_pub.publish(cmd);
                running = false;
                ros::spinOnce();
                control_rate.sleep();
            } break;

            default: {
            } break;
        }
    }
    return 0;
}

void generate_path_RRT()
{
    //TODO
    std::vector< std::vector<traj> > path_temp;    // save path of subTree
    for (int i = 0; i < waypoints.size()-1; i++)
    {
        rrtTree subtree;
//	std::cout <<"from: "<<waypoints[i].x <<" "<<waypoints[i].y << " " << waypoints[i].th << "   to: "<<waypoints[i+1].x <<" "<<waypoints[i+1].y<<" "<< waypoints[i+1].th << std::endl; // for debug
	subtree = rrtTree(waypoints[i], waypoints[i + 1], map, map_origin_x, map_origin_y, res, margin);
	
        int k = subtree.generateRRT(world_x_max, world_x_min, world_y_max, world_y_min, K, MaxStep);  // check RRT
	if(!k)	// RRT is not OK => delete previous RRT, and make it one more time
	{
	    if (i == 0)	// exception control
		i -= 1;
	    else
		{
		path_temp.pop_back();	// delete previous RRT
		i -= 2;
		}
	    continue;
	}
	if(k)	// RRT is OK
	{
	    std::cout << (i+1) << "th generateRRT is fine" << std::endl; // for debug
            std::vector<traj> route_reverse = subtree.backtracking_traj();

            //std::cout <<"subtre size: "<<route_reverse.size()<<std::endl;    // for debug
            std::reverse(route_reverse.begin(), route_reverse.end());

	    path_temp.push_back(route_reverse);	// save subTree's path
	    // set heading direction
	    waypoints[i+1].th = atan2(path_temp[i][path_temp[i].size()-1].y - path_temp[i][path_temp[i].size()-2].y, path_temp[i][path_temp[i].size()-1].x - path_temp[i][path_temp[i].size()-2].x);
	    //std::cout << waypoints[i+1].th << std::endl;    // for debug
            //std::cout<<std::endl;	// for debug
    	}
    }
	std::cout << "start printing" << std::endl;

    for (int i = 0 ; i < waypoints.size() - 1; i++)
    {
        for (int
 j = 0; j < path_temp[i].size(); j++)
	    path_RRT.push_back(path_temp[i][j]);
        std::cout << path_RRT.size() << std::endl; // for debug
    }
}



void set_waypoints()
{
    point waypoint_candid[5];
    waypoint_candid[0].x = -3.5;
    waypoint_candid[0].y = 12.0;

    waypoint_candid[1].x = 2.0;
    waypoint_candid[1].y = 12.0;
    
    waypoint_candid[2].x = 3.5;
    waypoint_candid[2].y = -10.5;
//    waypoint_candid[2].th = - 3.14/2;

    waypoint_candid[3].x = -2.0;
    waypoint_candid[3].y = -12.0;
//        waypoint_candid[3].th = 3.14;

    waypoint_candid[4].x = -3.5;
    waypoint_candid[4].y = 10.0;

    int order[] = {0,1,2,3,4};
    int order_size = 5;

    for(int i = 0; i < order_size; i++){
        waypoints.push_back(waypoint_candid[order[i]]);
    }
}

void set_angle()
{
    int i = 1;
    for (int i =1; i<waypoints.size()-1; i++)
    {
        waypoints[i].th = atan2(waypoints[i+1].y - waypoints[i-1].y, waypoints[i+1].x - waypoints[i-1].x);
//std::cout << waypoints[i+1].th << std::endl;
    }
    waypoints[i].th = atan2(waypoints[i].y - waypoints[i-1].y , waypoints[i].x - waypoints[i-1].x);
}

//from: 3.5 -10.5 -1.57   to: -2 -12 3.14

void callback_state(gazebo_msgs::ModelStatesConstPtr msgs){
    model_states = msgs;
    for(int i; i < msgs->name.size(); i++){
        if(std::strcmp(msgs->name[i].c_str(),"racecar") == 0){
            robot_pose.x = msgs->pose[i].position.x;
            robot_pose.y = msgs->pose[i].position.y;
            robot_pose.th = tf::getYaw(msgs->pose[i].orientation);
        }
    }
}

void setcmdvel(double vel, double deg){
    cmd.drive.speed = vel;
    cmd.drive.steering_angle = deg;
}
