//
// Created by gbuisan on 11/17/17.
//

#include "../include/HATEBMonitor.h"

HATEBMonitor::HATEBMonitor(ros::NodeHandle& n, ros::NodeHandle& pn): is_first_message(true), public_node_handle(n),
                                                                     private_node_handle(pn), lpf_average(0){
    time_to_goal_subscriber = n.subscribe<hanp_msgs::TimeToGoal>(TIME_TO_GOAL_TOPIC, 10, &HATEBMonitor::newTimeToGoalEstimateCallback, this);
    nominal_estimation_publisher = private_node_handle.advertise<teb_local_planner::OptimizationCost>(NOMINAL_ESTIMATION_PUB_SUBTOPIC, 10);
    eta_publisher = private_node_handle.advertise<std_msgs::Float64>(ETA_PUB_TOPIC, 10);
    filtered_eta_publisher = private_node_handle.advertise<std_msgs::Float64>(FILTERED_ETA_PUB_TOPIC, 10);
    ROS_INFO_NAMED(NODE_NAME, "Subscribed to %s", NODE_NAME);
}



void HATEBMonitor::newTimeToGoalEstimateCallback(const hanp_msgs::TimeToGoalConstPtr &msg) {
    std_msgs::Float64 eta_msg, filtered_eta_msg;
    double eta = msg->time_to_goal.toSec();
    eta_msg.data = eta;

    eta_publisher.publish(eta_msg);
    if (previous_msgs.size() == 0) {
        lpf_average = eta;
        previous_msgs.push(std::make_pair(msg->header.stamp, ros::Duration(eta)));

    }else {
        double filtered_eta = lowPassExponential(eta);
        filtered_eta_msg.data = filtered_eta;
        filtered_eta_publisher.publish(filtered_eta_msg);
        ros::Duration filtered_eta_duration = ros::Duration(filtered_eta);

        if (previous_msgs.size() >= NOMINAL_EST_WINDOW_SIZE) {
            std::pair<ros::Time, ros::Duration> old_msg = previous_msgs.front();

            ros::Duration difference_time_estimated = old_msg.second - filtered_eta_duration;
            ros::Duration difference_real_time = msg->header.stamp - old_msg.first;
            double q = difference_time_estimated.toSec() / difference_real_time.toSec();
            //double q = difference_time_estimated.toSec();

            teb_local_planner::OptimizationCost out_msg;
            out_msg.cost = q;
            nominal_estimation_publisher.publish(out_msg);
            previous_msgs.pop();
        }
        previous_msgs.push(std::make_pair(msg->header.stamp, filtered_eta_duration));
    }

    std::queue<std::pair<ros::Time, ros::Duration> > copy = previous_msgs;
    while (!copy.empty())
    {
        std::cout << copy.front().second << " ";
        copy.pop();
    }
    std::cout << std::endl;
}

double HATEBMonitor::lowPassExponential(double input) {
    lpf_average = input * LPF_FACTOR + (1-LPF_FACTOR)*lpf_average;
    return lpf_average;
}

HATEBMonitor::~HATEBMonitor() = default;