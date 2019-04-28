/*
 * MakePlan.cpp
 *
 *  Created on: Dec 9, 2013
 *      Author: roiyeho
 */

#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>

#include <string>
using std::string;

#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

double goalTolerance = 0.1;

void fillPathRequest(nav_msgs::GetPlan::Request &req);
void callPlanningService(ros::ServiceClient &serviceClient, nav_msgs::GetPlan &srv);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "make_plan_node");
	ros::NodeHandle nh;

	// Init service query for make plan
	string service_name = "move_base/make_plan";
	while (!ros::service::waitForService(service_name, ros::Duration(3.0))) {
		ROS_INFO("Waiting for service move_base/make_plan to become available");
	}

	ros::ServiceClient serviceClient = nh.serviceClient<nav_msgs::GetPlan>(service_name, true);
	if (!serviceClient) {
		ROS_FATAL("Could not initialize get plan service from %s", serviceClient.getService().c_str());
		return -1;
	}

	nav_msgs::GetPlan srv;
	fillPathRequest(srv.request);

	if (!serviceClient) {
		ROS_FATAL("Persistent service connection to %s failed", serviceClient.getService().c_str());
		return -1;
	}

	callPlanningService(serviceClient, srv);
}

void fillPathRequest(nav_msgs::GetPlan::Request &request)
{
	request.start.header.frame_id = "map";
	request.start.pose.position.x = 0;
	request.start.pose.position.y = 0;
	request.start.pose.orientation.w = 1.0;

	request.goal.header.frame_id = "map";
	request.goal.pose.position.x = 2.0;
	request.goal.pose.position.y = -2.0;
	request.goal.pose.orientation.w = 1.0;

	request.tolerance = goalTolerance;
}

void callPlanningService(ros::ServiceClient &serviceClient, nav_msgs::GetPlan &srv)
{
	if (serviceClient.call(srv)) {
		if (!srv.response.plan.poses.empty()) {
			forEach(const geometry_msgs::PoseStamped &p, srv.response.plan.poses) {
				ROS_INFO("x = %f, y = %f", p.pose.position.x, p.pose.position.y);
			}
		}
		else {
			ROS_WARN("Got empty plan");
		}
	}
	else {
		ROS_ERROR("Failed to call service %s - is the robot moving?", serviceClient.getService().c_str());
	}
}





