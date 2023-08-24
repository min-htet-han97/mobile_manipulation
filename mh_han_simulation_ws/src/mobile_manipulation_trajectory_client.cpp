/**
*
* ROS Simulation Online Course
* @uthor Min Htet Han (ROM Robotics)
* 
**/


#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <mh_han_simulation_ws/mobile_manipulationAction.h>
#include <vector>
#include <math.h>
#include <string>
#include <gazebo_msgs/GetJointProperties.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

geometry_msgs::Pose2D current_pos;
geometry_msgs::Pose2D desire_pos;
geometry_msgs::Twist cmd_vel;


double quatx;
double quaty;
double quatz;
double quatw;


double normalize1, normalize2, normalize_result1, normalize_result2;
double dx, dy, rho, alpha, beta, abs_v;
double v , w, v_const, w_const; // Linear and angular velocities
double constant_vel = 0.3;
double k_rho = 0.1;
double k_alpha = 0.15;//0.8, 0.5,0.4,0.2
double k_beta = -0.7; //0.15,0.9




std::vector<double> Arm_inverse_Kinematics(geometry_msgs::Point gripper_pos, double gripper_dist){

	double link1_height    = 1.0;   // distance from top of link1 to ground
	double link2_3_length  = 1.0;   // length of link2 & 3
	double gripper_center  = 0.15;  // distance from gripper center to link4
	double gripper_radius  = 0.04;  // the radius of the gripper paddle
	double gripper_range   = 0.16;  // distance from the gripper paddle axis to gripper center

	// convert input position to cylindric coordinates (cyl_r, cyl_theta, cyl_z)
	double cyl_r = sqrt(pow(gripper_pos.x, 2) + pow(gripper_pos.y, 2));
	double cyl_theta = atan(gripper_pos.y/gripper_pos.x);
	double cyl_z = gripper_pos.z;

	double theta1 = atan((cyl_z - link1_height)/(cyl_r - gripper_center));
	double theta2 = acos(sqrt(pow(cyl_z - link1_height, 2) + pow(cyl_r - gripper_center, 2))/2);
	double half_dist = gripper_dist/2 + gripper_radius;

	// output joints vector
	std::vector<double> cmd_jnts;
	cmd_jnts.resize(6);     // 6 joints including two gripper 

	cmd_jnts[0] = cyl_theta;// Joint_1
	cmd_jnts[1] = M_PI/2 - (theta1 + theta2); // Joint_2
	cmd_jnts[2] = M_PI - acos((2 - (pow(cyl_z - link1_height, 2) + pow(cyl_r - gripper_center, 2)))/2);//2*theta2; // Joint_3
	cmd_jnts[3] = M_PI/2 - cmd_jnts[1] - cmd_jnts[2]; // Joint_4, theta1 - theta2
	// prismatic joints: joint5 & 6
	cmd_jnts[4] = -(gripper_range - half_dist); // Joint_5
	cmd_jnts[5] = (gripper_range - half_dist); // Joint_6

	return cmd_jnts;

}

// void current_pos_Callback(const nav_msgs::Odometry::ConstPtr& msg){

// 	current_pos.x = msg->pose.pose.position.x;
// 	current_pos.y = msg->pose.pose.position.y;

// 	// Convertion from Quaternion to Euler anglue 
// 	tf::Quaternion q(
// 	msg->pose.pose.orientation.x,
// 	msg->pose.pose.orientation.y,
// 	msg->pose.pose.orientation.z,
// 	msg->pose.pose.orientation.w);
// 	tf::Matrix3x3 m(q);
// 	double roll, pitch, yaw;
// 	m.getRPY(roll, pitch, yaw);

// 	current_pos.theta = yaw;
// 	ROS_INFO("%f " , yaw);

// }

// void desire_pos_Callback(const geometry_msgs::Pose2D::ConstPtr& msg){
// 	desire_pos.x = msg->x;
// 	desire_pos.y = msg->y;
// 	//desire_pos.theta = msg->theta;
// }
// *****************************Mobile robot kinematics*************************************
// double get_error_angular(geometry_msgs::Pose2D desire_pos, geometry_msgs::Pose2D current_pos){
// 	// create error vector
// 	float error_x = desire_pos.x - current_pos.x; 
// 	float error_y = desire_pos.y - current_pos.y; 
	
// 	// get desire angle
// 	float desire_theta = atan2f(error_y, error_x);
	
// 	// get angle error
// 	float error_theta = desire_theta - current_pos.theta;
	
// 	//~ ROS_INFO("Ex: %f, Ey: %f, Et: %f", Ex, Ey, Et);
// 	return error_theta;
// }
double normalize(double angle){
	return atan2f(sin(angle), cos(angle));
}

// double get_error_linear(geometry_msgs::Pose2D desire_pos, geometry_msgs::Pose2D current_pos){
// 	// create error vector
// 	float error_x = desire_pos.x - current_pos.x;
// 	float error_y = desire_pos.y - current_pos.y; 
// 	float error_theta = get_error_angular(desire_pos, current_pos);	
	
// 	//~ float Etx =  pow( pow(Ex,2.0) + pow(Ey,2.0), 0.5 )*cos(Et);
// 	float error_linear = hypotf(error_x, error_y)*cos(error_theta); // improved c function
	
// 	return error_linear;

// }
//*******************************************************************************************



// callback to get "result" message from action server
void doneCb(const actionlib::SimpleClientGoalState& state,
		const mh_han_simulation_ws::mobile_manipulationResultConstPtr& result) {
	ROS_INFO("doneCb: server responded with state [%s]", state.toString().c_str());
}

int main(int argc, char** argv){
	ros::init(argc, argv, "mobile_manipulation_trajectory_client_node");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",10);

	double dt_sample = 1.0;
	int time = 5;  // Time for robot arm's joints movement

	double time_delay = 1.0;           // Delay between every tesk
	double lift_height = 0.3;          // height to lift up
	std::vector<double> start_jnts;    // start joints for each move task
	std::vector<double> start_joints_mobile_robot;
	std::vector<double> end_joints_mobile_robot;
	std::vector<double> end_jnts;      // end joints for each move task
	double fraction_of_range;
	bool finish_before_timeout;
	start_jnts.resize(6);
	end_jnts.resize(6);

	double beer_height = 0.28;   // 0.23 is real height, use 0.30 to lift the grasp point up a little
	double table_height = 1.0;   // the height of the table
	double gripper_open = 0.24;  // distance of two paddles when gripper is open
	double gripper_close = 0.06; // distance of two paddles when grasping the beer

	// initialize an action client for robot arm
	actionlib::SimpleActionClient<mh_han_simulation_ws::mobile_manipulationAction> action_client_robot_arm(
		"robot_arm_trajectory_action", true);
	actionlib::SimpleActionClient<mh_han_simulation_ws::mobile_manipulationAction> action_client_mobile_robot(
		"mobile_robot_trajectory_action", true);

	// Try to connect to action server!!!
	bool server_exist = action_client_robot_arm.waitForServer(ros::Duration(5.0));
	ros::Duration sleep1s(1);
	if(!server_exist) {
		ROS_WARN("could not connect to server; retrying");
		bool server_exist = action_client_robot_arm.waitForServer(ros::Duration(1.0));
		sleep1s.sleep();
	}
	// Client is connected to the server
	ROS_INFO("connected to action server");

	mh_han_simulation_ws::mobile_manipulationGoal goal_robot_arm;
	mh_han_simulation_ws::mobile_manipulationGoal goal_mobile_robot;

	// instantiate goal message
	trajectory_msgs::JointTrajectory trajectory_robot_arm;
	trajectory_msgs::JointTrajectory trajectory_mobile_robot;
	trajectory_msgs::JointTrajectoryPoint trajectory_points_robot_arm;
	trajectory_msgs::JointTrajectoryPoint trajectory_points_mobile_robot;

	// joint_names field for robot arm 
	trajectory_robot_arm.joint_names.resize(6);
	trajectory_robot_arm.joint_names[0] = "joint1";
	trajectory_robot_arm.joint_names[1] = "joint2";
	trajectory_robot_arm.joint_names[2] = "joint3";
	trajectory_robot_arm.joint_names[3] = "joint4";
	trajectory_robot_arm.joint_names[4] = "joint5";
	trajectory_robot_arm.joint_names[5] = "joint6";

	// joint_names field for mobile robot 
	trajectory_mobile_robot.joint_names.resize(4);
	trajectory_mobile_robot.joint_names[0] = "left_back_wheel_joint";
	trajectory_mobile_robot.joint_names[1] = "left_front_wheel_joint";
	trajectory_mobile_robot.joint_names[2] = "right_back_wheel_joint";
	trajectory_mobile_robot.joint_names[3] = "right_front_wheel_joint";

	// positions and velocities field
	trajectory_points_robot_arm.positions.resize(6);
	trajectory_points_mobile_robot.velocities.resize(4);

	// initialize a service client to get joint positions from gazebo simulation
	ros::ServiceClient get_jnt_state_client = nh.serviceClient<gazebo_msgs::GetJointProperties>(
		"/gazebo/get_joint_properties");
	gazebo_msgs::GetJointProperties get_joint_state_srv_msg;
	// initialize a service client to get model state
	ros::ServiceClient get_model_state_client = nh.serviceClient<gazebo_msgs::GetModelState>(
		"/gazebo/get_model_state");
	gazebo_msgs::GetModelState get_model_state_srv_msg;

	// Starting mobile robot position control
    
	// geometry_msgs::Point beer_pos; // {float64 x, float64 y, float z} Get the position of beer in gazebo simulation
	// get_model_state_srv_msg.request.model_name = "beer";
	// get_model_state_srv_msg.request.relative_entity_name = "link"; // Global frame of gazebo simulation
	// get_model_state_client.call(get_model_state_srv_msg);
	// beer_pos = get_model_state_srv_msg.response.pose.position; This is true!!!

	nav_msgs::Odometry beer_pos; // {float64 x, float64 y, float z} Get the position of beer in gazebo simulation
	get_model_state_srv_msg.request.model_name = "beer";
	get_model_state_srv_msg.request.relative_entity_name = "link"; // Global frame of gazebo simulation
	get_model_state_client.call(get_model_state_srv_msg);
	beer_pos.pose.pose.position.x = get_model_state_srv_msg.response.pose.position.x;
	beer_pos.pose.pose.position.y = get_model_state_srv_msg.response.pose.position.y;
	beer_pos.pose.pose.orientation.x = get_model_state_srv_msg.response.pose.orientation.x;
	beer_pos.pose.pose.orientation.y = get_model_state_srv_msg.response.pose.orientation.y;
	beer_pos.pose.pose.orientation.z = get_model_state_srv_msg.response.pose.orientation.z;
	beer_pos.pose.pose.orientation.w = get_model_state_srv_msg.response.pose.orientation.w;
	desire_pos.x = beer_pos.pose.pose.position.x;
	desire_pos.y = beer_pos.pose.pose.position.y;
	//Convertion from Quaternion to Euler anglue 
	tf::Quaternion beer_(
	beer_pos.pose.pose.orientation.x,
	beer_pos.pose.pose.orientation.y,
	beer_pos.pose.pose.orientation.z,
	beer_pos.pose.pose.orientation.w);
	tf::Matrix3x3 b(beer_);
	double roll, pitch, yaw;
	b.getRPY(roll, pitch, yaw);
	desire_pos.theta = yaw;

	

	// Get the current position of mobile robot in gazebo
 	nav_msgs::Odometry mobile_robot_pos;
	get_model_state_srv_msg.request.model_name = "mobile_manipulation";
	get_model_state_srv_msg.request.relative_entity_name = "link";
	get_model_state_client.call(get_model_state_srv_msg);	
	mobile_robot_pos.pose.pose.position.x = get_model_state_srv_msg.response.pose.position.x;
	mobile_robot_pos.pose.pose.position.y = get_model_state_srv_msg.response.pose.position.y;
	mobile_robot_pos.pose.pose.orientation.x = get_model_state_srv_msg.response.pose.orientation.x;
	mobile_robot_pos.pose.pose.orientation.y = get_model_state_srv_msg.response.pose.orientation.y;
	mobile_robot_pos.pose.pose.orientation.z = get_model_state_srv_msg.response.pose.orientation.z;
	mobile_robot_pos.pose.pose.orientation.w = get_model_state_srv_msg.response.pose.orientation.w;
	// Mobile robot


	desire_pos.x = beer_pos.pose.pose.position.x;
	desire_pos.y = beer_pos.pose.pose.position.y;
	desire_pos.theta = yaw;
	if (desire_pos.theta > M_PI){ desire_pos.theta -= 2*M_PI;}
	else if(desire_pos.theta < M_PI){ desire_pos.theta += 2*M_PI;}

	current_pos.x = mobile_robot_pos.pose.pose.position.x;
	current_pos.y = mobile_robot_pos.pose.pose.position.y;
	ROS_INFO("beer-%f and beer-%f and bot-%f and bot-%f", desire_pos.x , desire_pos.y, current_pos.x, current_pos.y );
	// Convertion from Quaternion to Euler anglue 
	tf::Quaternion mobile(
	mobile_robot_pos.pose.pose.orientation.x,
	mobile_robot_pos.pose.pose.orientation.y,
	mobile_robot_pos.pose.pose.orientation.z,
	mobile_robot_pos.pose.pose.orientation.w);
	tf::Matrix3x3 m(mobile);
	double mobile_roll, mobile_pitch, mobile_yaw;
	m.getRPY(mobile_roll, mobile_pitch, mobile_yaw);

	current_pos.theta = mobile_yaw;

	ROS_INFO("%f and %f and %f and %f", mobile_robot_pos.pose.pose.orientation.x, mobile_robot_pos.pose.pose.orientation.y, mobile_robot_pos.pose.pose.orientation.z, mobile_robot_pos.pose.pose.orientation.w);
	// double quatx= mobile_robot_pos.pose.pose.orientation.x;
 //    double quaty= mobile_robot_pos.pose.pose.orientation.y;
 //    double quatz= mobile_robot_pos.pose.pose.orientation.z;
 //    double quatw= mobile_robot_pos.pose.pose.orientation.w;

 //    tf::Quaternion q(quatx, quaty, quatz, quatw);
 //    tf::Matrix3x3 m(q);
 //    double roll, pitch, yaw;
 //    m.getRPY(roll, pitch, yaw);
 //    current_pos.theta = yaw;


	//double error_linear = 0.0;
	//double error_angular = 0.0;

	ROS_INFO("task 1: move the gripper to the safe point!!!");

	// get the original joint positions when this node is invoked
	std::vector<double> origin_jnts;
	origin_jnts.resize(6);
	for (int i=0; i<6; i++) {
		get_joint_state_srv_msg.request.joint_name = trajectory_robot_arm.joint_names[i];
		get_jnt_state_client.call(get_joint_state_srv_msg);
		origin_jnts[i] = get_joint_state_srv_msg.response.position[0];
	}
	// assign current joints to start joints
	start_jnts = origin_jnts;

	// define the safe point, avoid singularity at origin
	std::vector<double> safe_jnts;
	safe_jnts.resize(6);
	safe_jnts[0] = 0; // joint1, at its origin
	safe_jnts[1] = 30.0/180.0*M_PI; // joint2, a little bit forward
	safe_jnts[2] = 75.0/180.0*M_PI; // joint3, a little bit forward
	safe_jnts[3] = M_PI/2 - safe_jnts[1] - safe_jnts[2]; // joint4, parallel to the ground
	safe_jnts[4] = 0; // joint5, at its origin
	safe_jnts[5] = 0; // joint6, at its origin
	// assign the safe joints to end joints
	end_jnts = safe_jnts;

	// prepare the goal message
	trajectory_robot_arm.points.clear();
	for (int i=0; i<time+1; i++) { // there are time+1 points, including start and end
		fraction_of_range = (double)i/time; // cautious, convert to double
		for (int j=0; j<6; j++) { // there are 6 joints
			trajectory_points_robot_arm.positions[j] = start_jnts[j] + (end_jnts[j] - start_jnts[j])*fraction_of_range;
		}
		trajectory_points_robot_arm.time_from_start = ros::Duration((double)i);
		trajectory_robot_arm.points.push_back(trajectory_points_robot_arm);
	}

	// copy this trajectory into our action goal
	goal_robot_arm.mobile_manipulation_trajectory = trajectory_robot_arm;
	// send out the goal
	action_client_robot_arm.sendGoal(goal_robot_arm, &doneCb);
	// wait for expected duration plus some tolerance (2 seconds)
	finish_before_timeout = action_client_robot_arm.waitForResult(ros::Duration(time + 2.0));
	if (!finish_before_timeout) {
		ROS_WARN("task 1 is not done. (timeout)");
		return 0;
	}
	else {
		ROS_INFO("task 1 is done.");
	}
	// if here, task 1 is finished successfully
	ros::Duration(time_delay).sleep(); // delay before jumping to next task
	
	// while(desire_pos.x < current_pos.x - 0.1 and desire_pos.y < current_pos.y - 0.1){
	dx = desire_pos.x - current_pos.x;
	dy = desire_pos.y - current_pos.y;
	rho = sqrt (dx*dx + dy*dy);
	ROS_INFO("rho is %f", rho);
	
	
	
	

	
	//while(rho > 0.2){
	while((current_pos.x) != desire_pos.x-0.1 and (current_pos.y) != desire_pos.y-0.1){
		ROS_INFO("In first while loop");


		// get_model_state_srv_msg.request.model_name = "beer";
		// get_model_state_srv_msg.request.relative_entity_name = "link"; // Global frame of gazebo simulation
		// get_model_state_client.call(get_model_state_srv_msg);
		// beer_pos = get_model_state_srv_msg.response.pose.position;

		//nav_msgs::Odometry beer_pos; // {float64 x, float64 y, float z} Get the position of beer in gazebo simulation
		get_model_state_srv_msg.request.model_name = "beer";
		get_model_state_srv_msg.request.relative_entity_name = "link"; // Global frame of gazebo simulation
		get_model_state_client.call(get_model_state_srv_msg);
		beer_pos.pose.pose.position.x = get_model_state_srv_msg.response.pose.position.x;
		beer_pos.pose.pose.position.y = get_model_state_srv_msg.response.pose.position.y;
		beer_pos.pose.pose.orientation.x = get_model_state_srv_msg.response.pose.orientation.x;
		beer_pos.pose.pose.orientation.y = get_model_state_srv_msg.response.pose.orientation.y;
		beer_pos.pose.pose.orientation.z = get_model_state_srv_msg.response.pose.orientation.z;
		beer_pos.pose.pose.orientation.w = get_model_state_srv_msg.response.pose.orientation.w;

		// geometry_msgs::Point beer_pos_check; 
		// get_model_state_srv_msg.request.model_name = "beer";
		// get_model_state_srv_msg.request.relative_entity_name = "base_link"; 
		// get_model_state_client.call(get_model_state_srv_msg);
		// beer_pos_check = get_model_state_srv_msg.response.pose.position;

		nav_msgs::Odometry beer_pos_check; // {float64 x, float64 y, float z} Get the position of beer in gazebo simulation
		get_model_state_srv_msg.request.model_name = "beer";
		get_model_state_srv_msg.request.relative_entity_name = "base_link"; // Global frame of gazebo simulation
		get_model_state_client.call(get_model_state_srv_msg);
		beer_pos_check.pose.pose.position.x = get_model_state_srv_msg.response.pose.position.x;
		beer_pos_check.pose.pose.position.y = get_model_state_srv_msg.response.pose.position.y;
		beer_pos_check.pose.pose.orientation.x = get_model_state_srv_msg.response.pose.orientation.x;
		beer_pos_check.pose.pose.orientation.y = get_model_state_srv_msg.response.pose.orientation.y;
		beer_pos_check.pose.pose.orientation.z = get_model_state_srv_msg.response.pose.orientation.z;
		beer_pos_check.pose.pose.orientation.w = get_model_state_srv_msg.response.pose.orientation.w;

		get_model_state_srv_msg.request.model_name = "mobile_manipulation";
		get_model_state_srv_msg.request.relative_entity_name = "link";
		get_model_state_client.call(get_model_state_srv_msg);	
		mobile_robot_pos.pose.pose.position.x = get_model_state_srv_msg.response.pose.position.x;
		mobile_robot_pos.pose.pose.position.y = get_model_state_srv_msg.response.pose.position.y;
		mobile_robot_pos.pose.pose.orientation.x = get_model_state_srv_msg.response.pose.orientation.x;
		mobile_robot_pos.pose.pose.orientation.y = get_model_state_srv_msg.response.pose.orientation.y;
		mobile_robot_pos.pose.pose.orientation.z = get_model_state_srv_msg.response.pose.orientation.z;
		mobile_robot_pos.pose.pose.orientation.w = get_model_state_srv_msg.response.pose.orientation.w;

		// geometry_msgs::Point mobile_pos_check; 
		// get_model_state_srv_msg.request.model_name = "mobile_manipulation";
		// get_model_state_srv_msg.request.relative_entity_name = "base_link"; 
		// get_model_state_client.call(get_model_state_srv_msg);
		// mobile_pos_check = get_model_state_srv_msg.response.pose.position;
		nav_msgs::Odometry mobile_pos_check;
		get_model_state_srv_msg.request.model_name = "mobile_manipulation";
		get_model_state_srv_msg.request.relative_entity_name = "base_link";
		get_model_state_client.call(get_model_state_srv_msg);	
		mobile_pos_check.pose.pose.position.x = get_model_state_srv_msg.response.pose.position.x;
		mobile_pos_check.pose.pose.position.y = get_model_state_srv_msg.response.pose.position.y;
		mobile_pos_check.pose.pose.orientation.x = get_model_state_srv_msg.response.pose.orientation.x;
		mobile_pos_check.pose.pose.orientation.y = get_model_state_srv_msg.response.pose.orientation.y;
		mobile_pos_check.pose.pose.orientation.z = get_model_state_srv_msg.response.pose.orientation.z;
		mobile_pos_check.pose.pose.orientation.w = get_model_state_srv_msg.response.pose.orientation.w;

		if(fabs(beer_pos_check.pose.pose.position.x) - fabs(mobile_pos_check.pose.pose.position.x) <1.50 and fabs(beer_pos_check.pose.pose.position.y) - fabs(mobile_pos_check.pose.pose.position.y) <1.50){break;}

		ROS_INFO("beer: %f and %f and mobile: %f and %f", beer_pos_check.pose.pose.position.x, beer_pos_check.pose.pose.position.y, mobile_pos_check.pose.pose.position.x, mobile_pos_check.pose.pose.position.y);

		desire_pos.x = beer_pos.pose.pose.position.x;
		desire_pos.y = beer_pos.pose.pose.position.y;
		desire_pos.theta = yaw;
		if (desire_pos.theta > M_PI){ desire_pos.theta -= 2*M_PI;}
	    else if(desire_pos.theta < M_PI){ desire_pos.theta += 2*M_PI;}

		current_pos.x = mobile_robot_pos.pose.pose.position.x;
		current_pos.y = mobile_robot_pos.pose.pose.position.y;
		current_pos.theta = mobile_yaw;
		//Feed_back_kinematics_control
		dx = desire_pos.x - current_pos.x;
	    dy = desire_pos.y - current_pos.y;
	    rho = sqrt(dx*dx + dy*dy);
	    ROS_INFO("this line is 432 %f",rho);
	    normalize1 = atan2f(dy,dx) - desire_pos.theta;
	    normalize2 = current_pos.theta - atan2f(dy,dx);
	    alpha = normalize(normalize1);

	    ROS_INFO("Normalize_result_is: %f", alpha);
	    if (normalize1 > M_PI){ normalize1 -= 2*M_PI;}
	    else if(normalize1 < M_PI){ normalize1 += 2*M_PI;}
	    ROS_INFO("M_PI normalize is: %f and %f", M_PI, normalize1);

	    beta  = normalize(normalize2);
	    v     = k_rho * rho;
	    w     = k_alpha * alpha + k_beta * beta;
	    abs_v = fabs(v);
	    v_const = v/abs_v * constant_vel;
	    w_const = w/abs_v * constant_vel;
	    cmd_vel.linear.x = v;
	    cmd_vel.angular.z = w;
	    ROS_INFO("%f",v);

		//error_angular = get_error_angular(desire_pos, current_pos);
		//error_linear = get_error_linear(desire_pos, current_pos);


		// cmd_vel.linear.x = error_linear * 0.1;
		// cmd_vel.angular.z = error_angular * 0.2;
	    // quatx= mobile_robot_pos.pose.pose.orientation.x;
        // quaty= mobile_robot_pos.pose.pose.orientation.y;
        // quatz= mobile_robot_pos.pose.pose.orientation.z;
  //   	quatw= mobile_robot_pos.pose.pose.orientation.w;
		// tf::Quaternion q(quatx, quaty, quatz, quatw);
  //   	tf::Matrix3x3 m(q);
  //   	double roll, pitch, yaw;
  //   	m.getRPY(roll, pitch, yaw);
  //   	current_pos.theta = yaw;

		tf::Quaternion mobile(
		mobile_robot_pos.pose.pose.orientation.x,
		mobile_robot_pos.pose.pose.orientation.y,
		mobile_robot_pos.pose.pose.orientation.z,
		mobile_robot_pos.pose.pose.orientation.w);
		tf::Matrix3x3 m(mobile);
		mobile_roll, mobile_pitch, mobile_yaw;
		m.getRPY(mobile_roll, mobile_pitch, mobile_yaw);
		current_pos.theta = mobile_yaw;

		tf::Quaternion beer_(
		beer_pos.pose.pose.orientation.x,
		beer_pos.pose.pose.orientation.y,
		beer_pos.pose.pose.orientation.z,
		beer_pos.pose.pose.orientation.w);
		tf::Matrix3x3 b(beer_);
		//double roll, pitch, yaw;
		b.getRPY(roll, pitch, yaw);
		desire_pos.theta = yaw;

		// if(cmd_vel.linear.x > 1.2 || cmd_vel.angular.z > 1.0){
		// 	cmd_vel.linear.x = 0.8;
		// 	cmd_vel.angular.z = 0.5;
		// }
		// if(cmd_vel.linear.x < -1.2 || cmd_vel.angular.z < -1.0){
		// 	cmd_vel.linear.x = -0.8;
		// 	cmd_vel.angular.z = -0.5;}
		ROS_INFO("in while loop");


		pub.publish(cmd_vel); 
		ros::Duration(1).sleep();
		}

	// Converting Mobile Twist to Motor Velocitiex
	double right_back_wheel_velocity = (cmd_vel.angular.z * 1.3)/2 + cmd_vel.linear.x;  // cmd_vel.linear.x + cmd_vel.angular.z*width/2
	double right_front_wheel_velocity = (cmd_vel.angular.z * 1.3)/2 + cmd_vel.linear.x;
	double left_back_wheel_velocity = cmd_vel.linear.x * 2 - right_back_wheel_velocity; //cmd_vel.linear.x - cmd_vel.angular.z*width/2
	double left_front_wheel_velocity = cmd_vel.linear.x * 2 - right_back_wheel_velocity;
	ROS_INFO("%f and %f and %f and %f",left_back_wheel_velocity, left_front_wheel_velocity, right_back_wheel_velocity, right_front_wheel_velocity );

	ROS_INFO(" Move command to mobile robot to move target position!!!");
	std::vector<double> origin_joints_mobile_robot;
	origin_joints_mobile_robot.resize(4);
	cmd_vel.linear.x = 0.0;
	cmd_vel.angular.z = 0.0;
	pub.publish(cmd_vel);
	ros::Duration(time_delay).sleep();
	// for (int i=0; i<4; i++){
	// 	get_joint_state_srv_msg.request.joint_name = trajectory_mobile_robot.joint_names[i];
	// 	get_jnt_state_client.call(get_joint_state_srv_msg);
	// 	//origin_joints_mobile_robot[i] = get_joint_state_srv_msg.response.position[0];;
	// }
	start_joints_mobile_robot = origin_joints_mobile_robot;
	std::vector<double> wheel_velocity;
	wheel_velocity.resize(4);
	wheel_velocity[0] = left_back_wheel_velocity;
	wheel_velocity[1] = left_front_wheel_velocity;
	wheel_velocity[2] = right_back_wheel_velocity;
	wheel_velocity[3] = right_front_wheel_velocity;
	end_joints_mobile_robot = wheel_velocity;
	ROS_INFO("wheel_velocity : %f and %f and %f and %f", wheel_velocity[0], wheel_velocity[1], wheel_velocity[2], wheel_velocity[3]);

	
	ros::Duration(1).sleep(); // delay before jumping to next task


	ROS_INFO("task 2: move the gripper to the top area of the beer.");

	// get the position of the beer from gazebo
	// the position from gazebo is the bottom center of the beer
	geometry_msgs::Point beer_pos_for_arm; // {float64 x, float64 y, float z}
	get_model_state_srv_msg.request.model_name = "beer";
	get_model_state_srv_msg.request.relative_entity_name = "base_link";
	// "link" is the entity name when I add a beer in gazebo
	get_model_state_client.call(get_model_state_srv_msg);
	beer_pos_for_arm = get_model_state_srv_msg.response.pose.position;

	// calculate robot joints when the gripper is above the beer
	geometry_msgs::Point hover_open_start_pos;
	hover_open_start_pos = beer_pos_for_arm;
	hover_open_start_pos.x = hover_open_start_pos.x - 0.02 - 0.5;
	hover_open_start_pos.y = hover_open_start_pos.y - 0.02;
	hover_open_start_pos.z = hover_open_start_pos.z + beer_height/2 + lift_height + 0.43;
	std::vector<double> hover_open_start_jnts; // the corresponding joint position
	hover_open_start_jnts.resize(6);
	hover_open_start_jnts = Arm_inverse_Kinematics(hover_open_start_pos, gripper_open); // calculate the robot joints

	// assign the start joints and end joints
	start_jnts = safe_jnts; // start with last joints
	end_jnts = hover_open_start_jnts;

	// //////////////////////////////////////////////////////////////////////////////////////////////
	// ROS_INFO("start_jnts: %f, %f, %f, %f, %f, %f", start_jnts[0], start_jnts[1], start_jnts[2],
	// 	start_jnts[3], start_jnts[4], start_jnts[5]);
	// ROS_INFO("end_jnts: %f, %f, %f, %f, %f, %f", end_jnts[0], end_jnts[1], end_jnts[2],
	// 	end_jnts[3], end_jnts[4], end_jnts[5]);
	// //////////////////////////////////////////////////////////////////////////////////////////////

	// prepare the goal message
	trajectory_robot_arm.points.clear();
	for (int i=0; i<time+1; i++) { // there are time_2+1 points, including start and end
		fraction_of_range = (double)i/time;
		for (int j=0; j<6; j++) { // there are 6 joints
			trajectory_points_robot_arm.positions[j] = start_jnts[j] + (end_jnts[j] - start_jnts[j])*fraction_of_range;
		}
		trajectory_points_robot_arm.time_from_start = ros::Duration((double)i);
		trajectory_robot_arm.points.push_back(trajectory_points_robot_arm);
	}
	// copy this trajectory into our action goal
	goal_robot_arm.mobile_manipulation_trajectory = trajectory_robot_arm;
	// send out the goal
	action_client_robot_arm.sendGoal(goal_robot_arm, &doneCb);
	// wait for expected duration plus some tolerance (2 seconds)
	finish_before_timeout = action_client_robot_arm.waitForResult(ros::Duration(time + 2.0));
	if (!finish_before_timeout) {
		ROS_WARN("task 2 is not done. (timeout)");
		return 0;
	}
	else {
		ROS_INFO("task 2 is done.");
	}
	// if here, task 2 is finished successfully
	ros::Duration(time_delay).sleep(); // delay before jumping to next task

	ROS_INFO("task 3: move the gripper around the beer.");

	// get the position of the beer from gazebo
	// the position from gazebo is the bottom center of the beer
	//geometry_msgs::Point beer_pos; // {float64 x, float64 y, float z}
	get_model_state_srv_msg.request.model_name = "beer";
	get_model_state_srv_msg.request.relative_entity_name = "base_link";
	// "link" is the entity name when I add a beer in gazebo
	get_model_state_client.call(get_model_state_srv_msg);
	beer_pos_for_arm = get_model_state_srv_msg.response.pose.position;

	// calculate robot joints when the gripper is above the beer
	
	geometry_msgs::Point move_around_beer_pos;
	move_around_beer_pos = beer_pos_for_arm;
	move_around_beer_pos.x = move_around_beer_pos.x - 0.02 - 0.5;
	move_around_beer_pos.y = move_around_beer_pos.y - 0.02;
	move_around_beer_pos.z = move_around_beer_pos.z  + 0.1;
	std::vector<double> move_around_beer_jnts;
	move_around_beer_jnts.resize(6);
	move_around_beer_jnts = Arm_inverse_Kinematics(move_around_beer_pos, gripper_open); // calculate the robot joints

	// assign the start joints and end joints
	start_jnts = hover_open_start_jnts; // start with last joints
	end_jnts = move_around_beer_jnts;

	// //////////////////////////////////////////////////////////////////////////////////////////////
	// ROS_INFO("start_jnts: %f, %f, %f, %f, %f, %f", start_jnts[0], start_jnts[1], start_jnts[2],
	// 	start_jnts[3], start_jnts[4], start_jnts[5]);
	// ROS_INFO("end_jnts: %f, %f, %f, %f, %f, %f", end_jnts[0], end_jnts[1], end_jnts[2],
	// 	end_jnts[3], end_jnts[4], end_jnts[5]);
	// //////////////////////////////////////////////////////////////////////////////////////////////

	// prepare the goal message
	trajectory_robot_arm.points.clear();
	for (int i=0; i<time+1; i++) { // there are time_2+1 points, including start and end
		fraction_of_range = (double)i/time;
		for (int j=0; j<6; j++) { // there are 6 joints
			trajectory_points_robot_arm.positions[j] = start_jnts[j] + (end_jnts[j] - start_jnts[j])*fraction_of_range;
		}
		trajectory_points_robot_arm.time_from_start = ros::Duration((double)i);
		trajectory_robot_arm.points.push_back(trajectory_points_robot_arm);
	}
	// copy this trajectory into our action goal
	goal_robot_arm.mobile_manipulation_trajectory = trajectory_robot_arm;
	// send out the goal
	action_client_robot_arm.sendGoal(goal_robot_arm, &doneCb);
	// wait for expected duration plus some tolerance (2 seconds)
	finish_before_timeout = action_client_robot_arm.waitForResult(ros::Duration(time + 2.0));
	if (!finish_before_timeout) {
		ROS_WARN("task 3 is not done. (timeout)");
		return 0;
	}
	else {
		ROS_INFO("task 3 is done.");
	}
	// if here, task 2 is finished successfully
	ros::Duration(time_delay).sleep(); // delay before jumping to next task


	ROS_INFO("task 4: clamp the gripper to grasp the beer.");

	// calculate robot joints when grasp the beer
	geometry_msgs::Point grasp_close_start_pos;
	grasp_close_start_pos = move_around_beer_pos;
	std::vector<double> grasp_close_start_jnts; // the corresponding joint position
	grasp_close_start_jnts.resize(6);
	grasp_close_start_jnts = Arm_inverse_Kinematics(grasp_close_start_pos, gripper_close);

	// assign the start joints and end joints
	start_jnts = move_around_beer_jnts;
	end_jnts = grasp_close_start_jnts;

	// prepare the goal message
	trajectory_robot_arm.points.clear();
	for (int i=0; i<time+1; i++) { // there are time_4+1 points, including start and end
		fraction_of_range = (double)i/time;
		for (int j=0; j<6; j++) { // there are 6 joints
			trajectory_points_robot_arm.positions[j] = start_jnts[j] + (end_jnts[j] - start_jnts[j])*fraction_of_range;
		}
		trajectory_points_robot_arm.time_from_start = ros::Duration((double)i);
		trajectory_robot_arm.points.push_back(trajectory_points_robot_arm);
	}
	// copy this trajectory into our action goal
	goal_robot_arm.mobile_manipulation_trajectory = trajectory_robot_arm;
	// send out the goal
	action_client_robot_arm.sendGoal(goal_robot_arm, &doneCb);
	// wait for expected duration plus some tolerance (2 seconds)
	finish_before_timeout = action_client_robot_arm.waitForResult(ros::Duration(time + 2.0));
	if (!finish_before_timeout) {
		ROS_WARN("task 4 is not done. (timeout)");
		return 0;
	}
	else {
		ROS_INFO("task 4 is done.");
	}
	// if here, task 4 is finished successfully
	ros::Duration(time_delay).sleep(); // delay before jumping to next task


	ROS_INFO("task 5: move the gripper with the load to the target position.");

	// calculate robot joints when grasp the beer
	geometry_msgs::Point move_with_load_to_target_pos;
	move_with_load_to_target_pos.x = 1.0;
	move_with_load_to_target_pos.y = 0.0;
	move_with_load_to_target_pos.z = 1.0;
	std::vector<double> move_with_load_to_target_jnts; // the corresponding joint position
	move_with_load_to_target_jnts.resize(6);
	move_with_load_to_target_jnts = Arm_inverse_Kinematics(move_with_load_to_target_pos, gripper_close);

	// assign the start joints and end joints
	start_jnts = grasp_close_start_jnts;
	end_jnts = move_with_load_to_target_jnts;

	// prepare the goal message
	trajectory_robot_arm.points.clear();
	for (int i=0; i<time+1; i++) { // there are time_4+1 points, including start and end
		fraction_of_range = (double)i/time;
		for (int j=0; j<6; j++) { // there are 6 joints
			trajectory_points_robot_arm.positions[j] = start_jnts[j] + (end_jnts[j] - start_jnts[j])*fraction_of_range;
		}
		trajectory_points_robot_arm.time_from_start = ros::Duration((double)i);
		trajectory_robot_arm.points.push_back(trajectory_points_robot_arm);
	}
	// copy this trajectory into our action goal
	goal_robot_arm.mobile_manipulation_trajectory = trajectory_robot_arm;
	// send out the goal
	action_client_robot_arm.sendGoal(goal_robot_arm, &doneCb);
	// wait for expected duration plus some tolerance (2 seconds)
	finish_before_timeout = action_client_robot_arm.waitForResult(ros::Duration(time + 2.0));
	if (!finish_before_timeout) {
		ROS_WARN("task 5 is not done. (timeout)");
		return 0;
	}
	else {
		ROS_INFO("task 5 is done.");
	}
	// if here, task 4 is finished successfully
	ros::Duration(time_delay).sleep(); // delay before jumping to next task


	// geometry_msgs::Point target_pos; // {float64 x, float64 y, float z} Get the position of beer in gazebo simulation
	// get_model_state_srv_msg.request.model_name = "brick_base_one";
	// get_model_state_srv_msg.request.relative_entity_name = "link"; // Global frame of gazebo simulation
	// get_model_state_client.call(get_model_state_srv_msg);
	// target_pos = get_model_state_srv_msg.response.pose.position;
	// desire_pos.x = target_pos.x - 1.0;
	// desire_pos.y = target_pos.y - 1.0;
	// desire_pos.theta = 0.0;


	// get_model_state_srv_msg.request.model_name = "mobile_manipulation";
	// get_model_state_srv_msg.request.relative_entity_name = "link";
	// get_model_state_client.call(get_model_state_srv_msg);	
	// mobile_robot_pos.pose.pose.position.x = get_model_state_srv_msg.response.pose.position.x;
	// mobile_robot_pos.pose.pose.position.y = get_model_state_srv_msg.response.pose.position.y;
	// mobile_robot_pos.pose.pose.orientation.x = get_model_state_srv_msg.response.pose.orientation.x;
	// mobile_robot_pos.pose.pose.orientation.y = get_model_state_srv_msg.response.pose.orientation.y;
	// mobile_robot_pos.pose.pose.orientation.z = get_model_state_srv_msg.response.pose.orientation.z;
	// mobile_robot_pos.pose.pose.orientation.w = get_model_state_srv_msg.response.pose.orientation.w;

	// current_pos.x = mobile_robot_pos.pose.pose.position.x;
	// current_pos.y = mobile_robot_pos.pose.pose.position.y;

	
	// //m(mobile);
	// //mobile_roll, mobile_pitch, mobile_yaw;
	// m.getRPY(mobile_roll, mobile_pitch, mobile_yaw);

	// current_pos.theta = mobile_yaw;

 // 	// quatx= mobile_robot_pos.pose.pose.orientation.x;
 //  //   quaty= mobile_robot_pos.pose.pose.orientation.y;
 //  //   quatz= mobile_robot_pos.pose.pose.orientation.z;
 //  //   quatw= mobile_robot_pos.pose.pose.orientation.w;

 //  //   tf::Quaternion quaternion(quatx, quaty, quatz, quatw);
 //  //   tf::Matrix3x3 matrix(q);
 //  //   double roll1, pitch1, yaw1;
 //  //   matrix.getRPY(roll1, pitch1, yaw1);
 //  //   current_pos.theta = yaw1;


	// //error_linear = 0.0;
	// //error_angular = 0.0;

	nav_msgs::Odometry brick_pos; // {float64 x, float64 y, float z} Get the position of beer in gazebo simulation
	get_model_state_srv_msg.request.model_name = "brick_base_one";
	get_model_state_srv_msg.request.relative_entity_name = "link"; // Global frame of gazebo simulation
	get_model_state_client.call(get_model_state_srv_msg);
	brick_pos.pose.pose.position.x = get_model_state_srv_msg.response.pose.position.x;
	brick_pos.pose.pose.position.y = get_model_state_srv_msg.response.pose.position.y;
	brick_pos.pose.pose.orientation.x = get_model_state_srv_msg.response.pose.orientation.x;
	brick_pos.pose.pose.orientation.y = get_model_state_srv_msg.response.pose.orientation.y;
	brick_pos.pose.pose.orientation.z = get_model_state_srv_msg.response.pose.orientation.z;
	brick_pos.pose.pose.orientation.w = get_model_state_srv_msg.response.pose.orientation.w;
	desire_pos.x = brick_pos.pose.pose.position.x;
	desire_pos.y = brick_pos.pose.pose.position.y;
	//Convertion from Quaternion to Euler anglue 
	tf::Quaternion brick_(
	brick_pos.pose.pose.orientation.x,
	brick_pos.pose.pose.orientation.y,
	brick_pos.pose.pose.orientation.z,
	brick_pos.pose.pose.orientation.w);
	tf::Matrix3x3 brick(brick_);
	double brick_roll, brick_pitch, brick_yaw;
	brick.getRPY(brick_roll, brick_pitch, brick_yaw);
	//desire_pos.theta = brick_yaw;

	

	// Get the current position of mobile robot in gazebo
 	//nav_msgs::Odometry mobile_robot_pos;
	get_model_state_srv_msg.request.model_name = "mobile_manipulation";
	get_model_state_srv_msg.request.relative_entity_name = "link";
	get_model_state_client.call(get_model_state_srv_msg);	
	mobile_robot_pos.pose.pose.position.x = get_model_state_srv_msg.response.pose.position.x;
	mobile_robot_pos.pose.pose.position.y = get_model_state_srv_msg.response.pose.position.y;
	mobile_robot_pos.pose.pose.orientation.x = get_model_state_srv_msg.response.pose.orientation.x;
	mobile_robot_pos.pose.pose.orientation.y = get_model_state_srv_msg.response.pose.orientation.y;
	mobile_robot_pos.pose.pose.orientation.z = get_model_state_srv_msg.response.pose.orientation.z;
	mobile_robot_pos.pose.pose.orientation.w = get_model_state_srv_msg.response.pose.orientation.w;
	// Mobile robot


	desire_pos.x = brick_pos.pose.pose.position.x;
	desire_pos.y = brick_pos.pose.pose.position.y;
	desire_pos.theta = brick_yaw;
	if (desire_pos.theta > M_PI){ desire_pos.theta -= 2*M_PI;}
	else if(desire_pos.theta < M_PI){ desire_pos.theta += 2*M_PI;}

	current_pos.x = mobile_robot_pos.pose.pose.position.x;
	current_pos.y = mobile_robot_pos.pose.pose.position.y;
	ROS_INFO("beer-%f and beer-%f and bot-%f and bot-%f", desire_pos.x , desire_pos.y, current_pos.x, current_pos.y );
	// Convertion from Quaternion to Euler anglue 
	// tf::Quaternion mobile(
	// mobile_robot_pos.pose.pose.orientation.x,
	// mobile_robot_pos.pose.pose.orientation.y,
	// mobile_robot_pos.pose.pose.orientation.z,
	// mobile_robot_pos.pose.pose.orientation.w);
	// m(mobile);
	// mobile_roll, mobile_pitch, mobile_yaw;
	m.getRPY(mobile_roll, mobile_pitch, mobile_yaw);

	current_pos.theta = mobile_yaw;

	dx = desire_pos.x - current_pos.x;
	dy = desire_pos.y - current_pos.y;
	rho = sqrt (dx*dx + dy*dy);
	ROS_INFO("rho is %f", rho);

	//ROS_INFO("%f and %f and %f and %f", mobile_robot_pos.pose.pose.orientation.x, mobile_robot_pos.pose.pose.orientation.y, mobile_robot_pos.pose.pose.orientation.z, mobile_robot_pos.pose.pose.orientation.w);

while((current_pos.x) != desire_pos.x- 1.0 and (current_pos.y) != desire_pos.y- 1.0){


		// get_model_state_srv_msg.request.model_name = "brick_base_one";
		// get_model_state_srv_msg.request.relative_entity_name = "link"; // Global frame of gazebo simulation
		// get_model_state_client.call(get_model_state_srv_msg);
		// target_pos = get_model_state_srv_msg.response.pose.position;
		//nav_msgs::Odometry brick_pos;
		get_model_state_srv_msg.request.model_name = "brick_base_one";
		get_model_state_srv_msg.request.relative_entity_name = "link";
		get_model_state_client.call(get_model_state_srv_msg);	
		brick_pos.pose.pose.position.x = get_model_state_srv_msg.response.pose.position.x;
		brick_pos.pose.pose.position.y = get_model_state_srv_msg.response.pose.position.y;
		brick_pos.pose.pose.orientation.x = get_model_state_srv_msg.response.pose.orientation.x;
		brick_pos.pose.pose.orientation.y = get_model_state_srv_msg.response.pose.orientation.y;
		brick_pos.pose.pose.orientation.z = get_model_state_srv_msg.response.pose.orientation.z;
		brick_pos.pose.pose.orientation.w = get_model_state_srv_msg.response.pose.orientation.w;

		// geometry_msgs::Point brick_pos_check; 
		// get_model_state_srv_msg.request.model_name = "brick_base_one";
		// get_model_state_srv_msg.request.relative_entity_name = "base_link"; 
		// get_model_state_client.call(get_model_state_srv_msg);
		// brick_pos_check = get_model_state_srv_msg.response.pose.position;
		nav_msgs::Odometry brick_pos_check;
		get_model_state_srv_msg.request.model_name = "brick_base_one";
		get_model_state_srv_msg.request.relative_entity_name = "base_link";
		get_model_state_client.call(get_model_state_srv_msg);	
		brick_pos_check.pose.pose.position.x = get_model_state_srv_msg.response.pose.position.x;
		brick_pos_check.pose.pose.position.y = get_model_state_srv_msg.response.pose.position.y;
		brick_pos_check.pose.pose.orientation.x = get_model_state_srv_msg.response.pose.orientation.x;
		brick_pos_check.pose.pose.orientation.y = get_model_state_srv_msg.response.pose.orientation.y;
		brick_pos_check.pose.pose.orientation.z = get_model_state_srv_msg.response.pose.orientation.z;
		brick_pos_check.pose.pose.orientation.w = get_model_state_srv_msg.response.pose.orientation.w;

		get_model_state_srv_msg.request.model_name = "mobile_manipulation";
		get_model_state_srv_msg.request.relative_entity_name = "link";
		get_model_state_client.call(get_model_state_srv_msg);	
		mobile_robot_pos.pose.pose.position.x = get_model_state_srv_msg.response.pose.position.x;
		mobile_robot_pos.pose.pose.position.y = get_model_state_srv_msg.response.pose.position.y;
		mobile_robot_pos.pose.pose.orientation.x = get_model_state_srv_msg.response.pose.orientation.x;
		mobile_robot_pos.pose.pose.orientation.y = get_model_state_srv_msg.response.pose.orientation.y;
		mobile_robot_pos.pose.pose.orientation.z = get_model_state_srv_msg.response.pose.orientation.z;
		mobile_robot_pos.pose.pose.orientation.w = get_model_state_srv_msg.response.pose.orientation.w;

		// geometry_msgs::Point mobile_pos_check; 
		// get_model_state_srv_msg.request.model_name = "mobile_manipulation";
		// get_model_state_srv_msg.request.relative_entity_name = "base_link"; 
		// get_model_state_client.call(get_model_state_srv_msg);
		// mobile_pos_check = get_model_state_srv_msg.response.pose.position;

		nav_msgs::Odometry mobile_pos_check;
		get_model_state_srv_msg.request.model_name = "mobile_manipulation";
		get_model_state_srv_msg.request.relative_entity_name = "base_link";
		get_model_state_client.call(get_model_state_srv_msg);	
		mobile_pos_check.pose.pose.position.x = get_model_state_srv_msg.response.pose.position.x;
		mobile_pos_check.pose.pose.position.y = get_model_state_srv_msg.response.pose.position.y;
		mobile_pos_check.pose.pose.orientation.x = get_model_state_srv_msg.response.pose.orientation.x;
		mobile_pos_check.pose.pose.orientation.y = get_model_state_srv_msg.response.pose.orientation.y;
		mobile_pos_check.pose.pose.orientation.z = get_model_state_srv_msg.response.pose.orientation.z;
		mobile_pos_check.pose.pose.orientation.w = get_model_state_srv_msg.response.pose.orientation.w;

		if(fabs(brick_pos_check.pose.pose.position.x) - fabs(mobile_pos_check.pose.pose.position.x) < 1.50 and fabs(brick_pos_check.pose.pose.position.y) - fabs(mobile_pos_check.pose.pose.position.y) < 1.50){break;}

		ROS_INFO("brick: %f and %f and mobile: %f and %f", brick_pos_check.pose.pose.position.x, brick_pos_check.pose.pose.position.y, mobile_pos_check.pose.pose.position.x, mobile_pos_check.pose.pose.position.y);

		desire_pos.x = brick_pos.pose.pose.position.x;
		desire_pos.y = brick_pos.pose.pose.position.y;
		desire_pos.theta = brick_yaw;
		if (desire_pos.theta > M_PI){ desire_pos.theta -= 2*M_PI;}
	    else if(desire_pos.theta < M_PI){ desire_pos.theta += 2*M_PI;}

		current_pos.x = mobile_robot_pos.pose.pose.position.x;
		current_pos.y = mobile_robot_pos.pose.pose.position.y;
		current_pos.theta = mobile_yaw;
		//Feed_back_kinematics_control
		dx = desire_pos.x - current_pos.x;
	    dy = desire_pos.y - current_pos.y;
	    rho = sqrt(dx*dx + dy*dy);
	    ROS_INFO("this line is 432 %f",rho);
	    normalize1 = atan2f(dy,dx) - desire_pos.theta;
	    normalize2 = current_pos.theta - atan2f(dy,dx);
	    alpha = normalize(normalize1);
	    beta  = normalize(normalize2);
	    v     = k_rho * rho;
	    w     = k_alpha * alpha + k_beta * beta;
	    abs_v = fabs(v);
	    v_const = v/abs_v * constant_vel;
	    w_const = w/abs_v * constant_vel;
	    cmd_vel.linear.x = v;
	    cmd_vel.angular.z = w;
	    ROS_INFO("%f",v);

	    pub.publish(cmd_vel); 

	    tf::Quaternion mobile(
		mobile_robot_pos.pose.pose.orientation.x,
		mobile_robot_pos.pose.pose.orientation.y,
		mobile_robot_pos.pose.pose.orientation.z,
		mobile_robot_pos.pose.pose.orientation.w);
		tf::Matrix3x3 m(mobile);
		mobile_roll, mobile_pitch, mobile_yaw;
		m.getRPY(mobile_roll, mobile_pitch, mobile_yaw);
		current_pos.theta = mobile_yaw;

		tf::Quaternion brick_(
		beer_pos.pose.pose.orientation.x,
		beer_pos.pose.pose.orientation.y,
		beer_pos.pose.pose.orientation.z,
		beer_pos.pose.pose.orientation.w);
		tf::Matrix3x3 brick(brick_);
		double brick_roll, brick_pitch, brick_yaw;
		brick.getRPY(brick_roll, brick_pitch, brick_yaw);
		desire_pos.theta = brick_yaw;

		
		
		
		// quatx= mobile_robot_pos.pose.pose.orientation.x;
  //   	quaty= mobile_robot_pos.pose.pose.orientation.y;
  //    	quatz= mobile_robot_pos.pose.pose.orientation.z;
  //   	quatw= mobile_robot_pos.pose.pose.orientation.w;
		// tf::Quaternion quaternion(quatx, quaty, quatz, quatw);
  //   	tf::Matrix3x3 matrix(q);
  //   	double roll1, pitch1, yaw1;
  //   	matrix.getRPY(roll1, pitch1, yaw1);
  //   	current_pos.theta = yaw1;


		//error_angular = get_error_angular(desire_pos, current_pos);
		//error_linear = get_error_linear(desire_pos, current_pos);

		//  cmd_vel.linear.x = error_linear * 0.1;
		//  cmd_vel.angular.z = error_angular * 0.2;

		// if(cmd_vel.linear.x > 1.2 || cmd_vel.angular.z > 1.0){
		// 	cmd_vel.linear.x = 0.8;
		// 	cmd_vel.angular.z = 0.5;
		// }
		// if(cmd_vel.linear.x < -1.2 || cmd_vel.angular.z < -1.0){
		// 	cmd_vel.linear.x = -0.8;
		// 	cmd_vel.angular.z = -0.5;}
		ROS_INFO("in second while loop");


		
		ros::Duration(1).sleep();
		}
		cmd_vel.linear.x = 0.0;
		cmd_vel.angular.z = 0.0;
		pub.publish(cmd_vel);

		


	ROS_INFO("task 6: move the gripper to the above of target area.");

	// get the position of the table from gazebo
	
	geometry_msgs::Point target_area;
	get_model_state_srv_msg.request.model_name = "brick_base_one";
	get_model_state_srv_msg.request.relative_entity_name = "base_link"; 
	get_model_state_client.call(get_model_state_srv_msg);
	target_area = get_model_state_srv_msg.response.pose.position;

	
	geometry_msgs::Point gripper_target_pos;
	gripper_target_pos.x = target_area.x - 0.5 ;
	gripper_target_pos.y = target_area.y ;
	gripper_target_pos.z = target_area.z + 0.2;

	// calculate the robot joints at the above of target area
	geometry_msgs::Point hover_close_end_pos;
	hover_close_end_pos = gripper_target_pos;
	//hover_close_end_pos.z = 1.30;
	std::vector<double> hover_close_end_jnts;
	hover_close_end_jnts.resize(6);
	hover_close_end_jnts = Arm_inverse_Kinematics(hover_close_end_pos, gripper_close);

	// assign the start joints and end joints
	start_jnts = move_with_load_to_target_jnts;
	end_jnts = hover_close_end_jnts;
	// prepare the goal message
	trajectory_robot_arm.points.clear();
	for (int i=0; i<time+1; i++) { // there are time_6+1 points, including start and end
		fraction_of_range = (double)i/time;
		for (int j=0; j<6; j++) { // there are 6 joints
			trajectory_points_robot_arm.positions[j] = start_jnts[j] + (end_jnts[j] - start_jnts[j])*fraction_of_range;
		}
		trajectory_points_robot_arm.time_from_start = ros::Duration((double)i);
		trajectory_robot_arm.points.push_back(trajectory_points_robot_arm);
	}
	// copy this trajectory into our action goal
	goal_robot_arm.mobile_manipulation_trajectory = trajectory_robot_arm;
	// send out the goal
	action_client_robot_arm.sendGoal(goal_robot_arm, &doneCb);
	// wait for expected duration plus some tolerance (2 seconds)
	finish_before_timeout = action_client_robot_arm.waitForResult(ros::Duration(time + 2.0));
	if (!finish_before_timeout) {
		ROS_WARN("task 6 is not done. (timeout)");
		return 0;
	}
	else {
		ROS_INFO("task 6 is done.");
	}
	// if here, task 6 is finished successfully
	ros::Duration(time_delay).sleep(); // delay before jumping to next task


	ROS_INFO("task 7: release the load.");

	// get the position of the table from gazebo
	
	
	geometry_msgs::Point gripper_release_pos;
	gripper_release_pos = hover_close_end_pos;

	// calculate the robot joints at the above of target area
	
	std::vector<double> gripper_release_jnts;
	gripper_release_jnts.resize(6);
	gripper_release_jnts = Arm_inverse_Kinematics(hover_close_end_pos, gripper_open);

	// assign the start joints and end joints
	start_jnts = hover_close_end_jnts;
	end_jnts = gripper_release_jnts;
	// prepare the goal message
	trajectory_robot_arm.points.clear();
	for (int i=0; i<time+1; i++) { // there are time_6+1 points, including start and end
		fraction_of_range = (double)i/time;
		for (int j=0; j<6; j++) { // there are 6 joints
			trajectory_points_robot_arm.positions[j] = start_jnts[j] + (end_jnts[j] - start_jnts[j])*fraction_of_range;
		}
		trajectory_points_robot_arm.time_from_start = ros::Duration((double)i);
		trajectory_robot_arm.points.push_back(trajectory_points_robot_arm);
	}
	// copy this trajectory into our action goal
	goal_robot_arm.mobile_manipulation_trajectory = trajectory_robot_arm;
	// send out the goal
	action_client_robot_arm.sendGoal(goal_robot_arm, &doneCb);
	// wait for expected duration plus some tolerance (2 seconds)
	finish_before_timeout = action_client_robot_arm.waitForResult(ros::Duration(time + 2.0));
	if (!finish_before_timeout) {
		ROS_WARN("task 7 is not done. (timeout)");
		return 0;
	}
	else {
		ROS_INFO("task 7 is done.");
	}
	// if here, task 6 is finished successfully
	ros::Duration(time_delay).sleep(); // delay before jumping to next task


	ROS_INFO("task 8: move to the safe position.");

	// calculate robot joints when grasp the beer
	geometry_msgs::Point move_safe_pos;
	move_safe_pos.x = 1.0;
	move_safe_pos.y = 0.0;
	move_safe_pos.z = 1.30;
	std::vector<double> move_safe_jnts; // the corresponding joint position
	move_safe_jnts.resize(6);
	move_safe_jnts = Arm_inverse_Kinematics(move_safe_pos, gripper_open);

	// assign the start joints and end joints
	start_jnts = gripper_release_jnts;
	end_jnts = move_safe_jnts;

	// prepare the goal message
	trajectory_robot_arm.points.clear();
	for (int i=0; i<time+1; i++) { // there are time_4+1 points, including start and end
		fraction_of_range = (double)i/time;
		for (int j=0; j<6; j++) { // there are 6 joints
			trajectory_points_robot_arm.positions[j] = start_jnts[j] + (end_jnts[j] - start_jnts[j])*fraction_of_range;
		}
		trajectory_points_robot_arm.time_from_start = ros::Duration((double)i);
		trajectory_robot_arm.points.push_back(trajectory_points_robot_arm);
	}
	// copy this trajectory into our action goal
	goal_robot_arm.mobile_manipulation_trajectory = trajectory_robot_arm;
	// send out the goal
	action_client_robot_arm.sendGoal(goal_robot_arm, &doneCb);
	// wait for expected duration plus some tolerance (2 seconds)
	finish_before_timeout = action_client_robot_arm.waitForResult(ros::Duration(time + 2.0));
	if (!finish_before_timeout) {
		ROS_WARN("task 8 is not done. (timeout)");
		return 0;
	}
	else {
		ROS_INFO("task 8 is done.");
	}
	// if here, task 4 is finished successfully
	ros::Duration(time_delay).sleep(); // delay before jumping to next task
	

	

	return 0;
}
