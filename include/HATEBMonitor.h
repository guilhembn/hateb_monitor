//
// Created by gbuisan on 11/17/17.
//

#ifndef HATEB_MONITOR_HATEBMONITOR_H
#define HATEB_MONITOR_HATEBMONITOR_H

#include <ros/ros.h>
#include "hanp_msgs/TimeToGoal.h"
#include "teb_local_planner/OptimizationCost.h"
#include "std_msgs/Float64.h"
#include <queue>

#define NODE_NAME "HatebMonitor"
#define OPTIMISATION_TOPIC "/TEBLocalPlanner/OptimisationCosts"
#define TIME_TO_GOAL_TOPIC "/move_base_node/TebLocalPlannerROS/traj_time"
#define NOMINAL_ESTIMATION_PUB_SUBTOPIC "nominal_estimation"
#define ETA_PUB_TOPIC "eta"

#define NOISE_THRESHOLD 5

class HATEBMonitor {
public:
    HATEBMonitor(ros::NodeHandle& n, ros::NodeHandle& pn);
    ~HATEBMonitor();

    const double LPF_FACTOR = 0.1;
    const int NOMINAL_EST_WINDOW_SIZE = 8;
    const std::string FILTERED_ETA_PUB_TOPIC = "filtered_eta";

    ros::NodeHandle public_node_handle;
    ros::NodeHandle private_node_handle;
    ros::Publisher nominal_estimation_publisher;
    ros::Publisher eta_publisher;
    ros::Publisher filtered_eta_publisher;
    //hanp_msgs::TimeToGoal previous_msg;
    double lpf_average;

    std::queue<std::pair<ros::Time, ros::Duration> > previous_msgs;
    bool is_first_message;
    ros::Subscriber time_to_goal_subscriber;
    double lowPassExponential(double input);


    void newTimeToGoalEstimateCallback(const hanp_msgs::TimeToGoalConstPtr& msg);

};


#endif //HATEB_MONITOR_HATEBMONITOR_H
