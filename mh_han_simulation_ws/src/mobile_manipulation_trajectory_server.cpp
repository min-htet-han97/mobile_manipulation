/**
*
* ROS Simulation Online Course
* @uthor Min Htet Han (ROM Robotics)
* 
**/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <mh_han_simulation_ws/mobile_manipulationAction.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <string>

const double dt = 0.01; //0.005; // resolution for interpolating the trajectory
const double dt_min = 0.0000001; // for examining the time step

class TrajectoryActionServer {
public:
	TrajectoryActionServer(ros::NodeHandle* nodehandle); // constructor
	~TrajectoryActionServer(void) {}  // destructor
private:
	void send_joint_commands_mobile_robot_(std::vector<double> cmd_jnts);
	void send_joint_commands_robot_arm_(std::vector<double> cmd_jnts);
	void executeCB_mobile_robot(const actionlib::SimpleActionServer<mh_han_simulation_ws::mobile_manipulationAction>::GoalConstPtr& goal_mobile_robot);
	void executeCB_robot_arm(const actionlib::SimpleActionServer<mh_han_simulation_ws::mobile_manipulationAction>::GoalConstPtr& goal_robot_arm);


	ros::NodeHandle nh_; // Nodehandle
	// Publisher for robot arm and mobile robot
	std::vector<ros::Publisher> pos_cmd_publisher_mobile_robot_; // initialization will be in executeCB_mobile_robot()
	std::vector<ros::Publisher> pos_cmd_publisher_robot_arm_;      // initialization will be in executeCB_robot_arm()
	actionlib::SimpleActionServer<mh_han_simulation_ws::mobile_manipulationAction> as_mobile_robot_;
	actionlib::SimpleActionServer<mh_han_simulation_ws::mobile_manipulationAction> as_robot_arm_;

	// message types for the action
	mh_han_simulation_ws::mobile_manipulationActionGoal goal_;           // goal message
	mh_han_simulation_ws::mobile_manipulationActionResult result_;       // result message
	mh_han_simulation_ws::mobile_manipulationActionFeedback feedback_;   // feedback message, not used

};


TrajectoryActionServer::TrajectoryActionServer(ros::NodeHandle* nodehandle):
	nh_(*nodehandle),    // dereference the pointer and pass the value
	as_mobile_robot_(nh_, "mobile_robot_trajectory_action", boost::bind(&TrajectoryActionServer::executeCB_mobile_robot, this, _1), false),
	as_robot_arm_(nh_, "robot_arm_trajectory_action", boost::bind(&TrajectoryActionServer::executeCB_robot_arm, this, _1), false)
	
{
	ROS_INFO("in constructor of TrajectoryActionServer...");
	as_mobile_robot_.start();   // start the mobile robot trajectory action server
 	as_robot_arm_.start();		// start the robot arm trajectory action server
	
}


// publish commands to all joint command topic
// this function will only be invoked in executeCB()


void TrajectoryActionServer::send_joint_commands_robot_arm_(std::vector<double> cmd_jnts) {
	int no_of_publishers = pos_cmd_publisher_robot_arm_.size();
	if (cmd_jnts.size() == no_of_publishers) { // dimension of pos_cmd and publisher is same
		for (int i=0; i<no_of_publishers; i++) {
			std_msgs::Float64 cmd_msg;
			cmd_msg.data = cmd_jnts[i];
			pos_cmd_publisher_robot_arm_[i].publish(cmd_msg);
		}
	}
	else
		ROS_WARN("joint commanders and publishers are not consistent!");
}

void TrajectoryActionServer::send_joint_commands_mobile_robot_(std::vector<double> cmd_jnts) {
	int no_of_publishers = pos_cmd_publisher_mobile_robot_.size();
	if (cmd_jnts.size() == no_of_publishers) { 
	 // dimension of pos_cmd and publisher is same
		for(int i=0; i < no_of_publishers; i++){
		float left_back_wheel_velocity = cmd_jnts[0];
		float left_front_wheel_velociy = cmd_jnts[1];
		float right_back_wheel_velocity = cmd_jnts[2];
		float right_front_wheel_velocity = cmd_jnts[3];
		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.x = (left_back_wheel_velocity + right_back_wheel_velocity) / 2;
		cmd_vel.angular.z = (right_back_wheel_velocity - cmd_vel.linear.x)*2 / 1.3;
		pos_cmd_publisher_mobile_robot_[i].publish(cmd_vel);
	}}
	else
		ROS_WARN("joint commanders and publishers are not consistent!");
	
}


void TrajectoryActionServer::executeCB_mobile_robot(const actionlib::SimpleActionServer<mh_han_simulation_ws::mobile_manipulationAction>::GoalConstPtr& goal_mobile_robot){
	ROS_INFO("in executeCB_mobile_robot...");
	trajectory_msgs::JointTrajectory trajectory_mobile_robot = goal_mobile_robot -> mobile_manipulation_trajectory;
	int no_of_joints = trajectory_mobile_robot.joint_names.size();
	int no_of_points = trajectory_mobile_robot.points.size();
	pos_cmd_publisher_mobile_robot_.resize(no_of_joints);
	for (int i=0; i < no_of_joints; i++){
		pos_cmd_publisher_mobile_robot_[i] = nh_.advertise<geometry_msgs::Twist>("cmd_vel",1,true);	

	}
	ROS_INFO("trajectory message commands mobile %d joint(s) and %d", no_of_joints, no_of_points);
	

		// joint array positions in the calculation of interpolation
	std::vector<double> prev_joints;  // previous joints in the interpolation
	std::vector<double> next_joints;  // next joints in the interpolation
	std::vector<double> cmd_joints;   // interpolated joints to be published
	prev_joints.resize(no_of_joints);   // resize to the number of joints
	next_joints.resize(no_of_joints);
	cmd_joints.resize(no_of_joints);

	
    // final time to finish these points //this is start from 0
	double t_final = trajectory_mobile_robot.points[no_of_points - 1].time_from_start.toSec();
		// the first point should be the current point in the gazebo
		// so the first time_from_start should be 0.0
	double t_previous = trajectory_mobile_robot.points[0].time_from_start.toSec();
	double t_next = trajectory_mobile_robot.points[1].time_from_start.toSec();
	double t_stream = t_previous; // initializee to t_previous
	double fraction_of_range = 0.0; // a ratio describing fraction of current segment completed

	// check the validality of time range
	double t_range = t_next - t_previous;
	if (t_range < dt_min) {
		ROS_WARN("time step invalid in trajectory! (begining)");
		
		as_mobile_robot_.setAborted();
		return;
	}

	int ipt_next = 1; // index these points, start from the first one
	// initialize joint values
	prev_joints = trajectory_mobile_robot.points[ipt_next - 1].positions;
	next_joints = trajectory_mobile_robot.points[ipt_next].positions;

	// start interpolation, use time to control this process
	ros::Rate rate_timer(1/dt);
	// Check current time exceeds final time or not
	while (t_stream < t_final) { 
		
		fraction_of_range = (t_stream - t_previous)/(t_next - t_previous);
		// linearly interpolating points for each joints
		for (int i=0; i< no_of_joints; i++) {
			cmd_joints[i] = prev_joints[i] + fraction_of_range*(next_joints[i] - prev_joints[i]);
			ROS_INFO("%f", cmd_joints[i]);
		}
		send_joint_commands_mobile_robot_(cmd_joints); // send these joint position to controller
		// for debug of these calculation, may comment out later
		// ROS_INFO("prev_jnts[2], next_jnts[2], cmd_jnts[2]: %f, %f, %f",
		// 	prev_jnts[2], next_jnts[2], cmd_jnts[2]);

		// time control
		rate_timer.sleep();
		t_stream = t_stream + dt;
	}

	// output the final point
	cmd_joints = trajectory_mobile_robot.points[no_of_points - 1].positions;
	send_joint_commands_mobile_robot_(cmd_joints);
	// as_.setSucceeded(result_); // finally successful on this action...
	as_mobile_robot_.setSucceeded();
}

void TrajectoryActionServer::executeCB_robot_arm(const actionlib::SimpleActionServer<mh_han_simulation_ws::mobile_manipulationAction>::GoalConstPtr& goal_robot_arm) {
	ROS_INFO("in executecb_robot_arm...");
	// avoid using "goal->trajectory" too much
	trajectory_msgs::JointTrajectory trajectory_robot_arm = goal_robot_arm -> mobile_manipulation_trajectory;
	int no_of_joints = trajectory_robot_arm.joint_names.size();  // get the number of joints
	int no_of_points = trajectory_robot_arm.points.size();       // get the number of points { push back }


	ROS_INFO("trajectory message commands %d joint(s) and %d", no_of_joints, no_of_points);
	//initialize command publishers for each joints
	pos_cmd_publisher_robot_arm_.resize(no_of_joints);
	for (int i=0; i < no_of_joints; i++) {
		pos_cmd_publisher_robot_arm_[i] = nh_.advertise<std_msgs::Float64>("/mobile_manipulation/" + trajectory_robot_arm.joint_names[i] + "_position_controller/command", 1, true);
	}

	// joint array positions in the calculation of interpolation
	std::vector<double> prev_joints;  // previous joints in the interpolation
	std::vector<double> next_joints;  // next joints in the interpolation
	std::vector<double> cmd_joints;   // interpolated joints to be published
	prev_joints.resize(no_of_joints);   // resize to the number of joints
	next_joints.resize(no_of_joints);
	cmd_joints.resize(no_of_joints);

	
    // final time to finish these points
	double t_final = trajectory_robot_arm.points[no_of_points - 1].time_from_start.toSec();
		// the first point should be the current point in the gazebo
		// so the first time_from_start should be 0.0
	double t_previous = trajectory_robot_arm.points[0].time_from_start.toSec();
	double t_next = trajectory_robot_arm.points[1].time_from_start.toSec();
	double t_stream = t_previous; // initializee to t_previous
	double fraction_of_range = 0.0; // a ratio describing fraction of current segment completed

	// check the validality of time range
	double t_range = t_next - t_previous;
	if (t_range < dt_min) {
		ROS_WARN("time step invalid in trajectory! (begining)");
		// as_.setAborted(result_);
		as_robot_arm_.setAborted();
		return;
	}

	int ipt_next = 1; // index these points, start from the first one
	// initialize joint values
	prev_joints = trajectory_robot_arm.points[ipt_next - 1].positions;
	next_joints = trajectory_robot_arm.points[ipt_next].positions;

	// start interpolation, use time to control this process
	ros::Rate rate_timer(1/dt);
	// Check current time exceeds final time or not
	while (t_stream < t_final) { 
		
		fraction_of_range = (t_stream - t_previous)/(t_next - t_previous);
		// linearly interpolating points for each joints
		for (int i=0; i<no_of_joints; i++) {
			cmd_joints[i] = prev_joints[i] + fraction_of_range*(next_joints[i] - prev_joints[i]);
		}
		send_joint_commands_robot_arm_(cmd_joints); // send these joint position to controller
		// for debug of these calculation, may comment out later
		// ROS_INFO("prev_jnts[2], next_jnts[2], cmd_jnts[2]: %f, %f, %f",
		// 	prev_jnts[2], next_jnts[2], cmd_jnts[2]);

		// time control
		rate_timer.sleep();
		t_stream = t_stream + dt;
	}

	// output the final point
	cmd_joints = trajectory_robot_arm.points[no_of_points - 1].positions;
	send_joint_commands_robot_arm_(cmd_joints);
	// as_.setSucceeded(result_); // finally successful on this action...
	as_robot_arm_.setSucceeded();
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "mobile_manipulation_trajectory_server_node");
	ros::NodeHandle nh;	// to be passed in the instantiation of class

	ROS_INFO("instantiating an object of class TrajectoryActionServer...");
	TrajectoryActionServer as_object(&nh);

	ROS_INFO("going into spin...");

	while (ros::ok()) {
		ros::spinOnce();
		ros::Duration(0.1).sleep();
	}

	return 0;
}

