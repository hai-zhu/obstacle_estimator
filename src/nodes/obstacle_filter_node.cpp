//
// Created by hai on 2/6/19.
//

#include <obstacle_estimator/obstacle_filter.h>

int main(int argc, char **argv)
{
    // Set up ROS
    ros::init(argc, argv, "obstacle_filter_node");  // node name
    ros::NodeHandle nh;                             // create a node handle

    // Set parameters
    double pub_rate;                               // publisher rate
    if (!nh.getParam("/pub_rate", pub_rate))
    {
        ROS_ERROR_STREAM("obstacle_filter_node Parameter " << ros::this_node::getName()+"/pub_rate not set");
        return 0;
    }

    std::string sub_topic;                          // subscriber topic name
    if (!nh.getParam("/obstacle/pose_measured", sub_topic))
    {
        ROS_ERROR_STREAM("obstacle_filter_node Parameter " << ros::this_node::getName()+"/pose_measured not set");
        return 0;
    }

    std::string pub_topic;                          // publisher topic name
    if (!nh.getParam("/obstacle/state_estimation", pub_topic))
    {
        ROS_ERROR_STREAM("obstacle_filter_node Parameter " << ros::this_node::getName()+"/state_estimation not set");
        return 0;
    }

    // Initialize a class object and pass node handle for constructor
    Obstacle_Filter obstacle_filter(nh, sub_topic, pub_topic);

    // Let ROS handle all callbacks
    ros::spin();

    return 0;
}