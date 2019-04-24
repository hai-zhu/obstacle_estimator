//
// Created by hai on 4/23/19.
//

#include <obstacle_estimator/obstacle_prediction.h>

int main(int argc, char **argv)
{
    // Set up ROS
    ros::init(argc, argv, "obstacle_prediction_node");  // node name
    ros::NodeHandle nh;                                 // create a node handle

    // Set parameters
    double node_rate;                                   // node rate
    if (!nh.getParam(ros::this_node::getName()+"/node_rate", node_rate))
    {
        ROS_ERROR_STREAM("obstacle_prediction_node Parameter " << ros::this_node::getName()+"/node_rate not set");
        return 0;
    }

    std::string sub_topic;                              // subscriber topic name
    if (!nh.getParam(ros::this_node::getName()+"/obstacle/pose_measured", sub_topic))
    {
        ROS_ERROR_STREAM("obstacle_prediction_node Parameter " << ros::this_node::getName()+"/pose_measured not set");
        return 0;
    }

    std::string pub_topic;                              // publisher topic name
    if (!nh.getParam(ros::this_node::getName()+"/obstacle/state_estimation", pub_topic))
    {
        ROS_ERROR_STREAM("obstacle_prediction_node Parameter " << ros::this_node::getName()+"/state_estimation not set");
        return 0;
    }

    double delta_t;                                     // delta t for prediction
    if (!nh.getParam(ros::this_node::getName()+"/delta_t", delta_t))
    {
        ROS_ERROR_STREAM("obstacle_prediction_node Parameter " << ros::this_node::getName()+"/delta_t not set");
        return 0;
    }

    int horizon_N;                                      // prediction horizon length
    if (!nh.getParam(ros::this_node::getName()+"/horizon_N", horizon_N))
    {
        ROS_ERROR_STREAM("obstacle_prediction_node Parameter " << ros::this_node::getName()+"/horizon_N not set");
        return 0;
    }

    // Initialize a class object and pass node handle for constructor
    Obstacle_Prediction obstacle_prediction(nh, sub_topic, pub_topic, node_rate, delta_t, horizon_N);

    // Let ROS handle all callbacks
    // specify the publishing rate
    // ros::Rate loop_rate(node_rate);
    // while (ros::ok())
    // {
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    // not specify the publishing rate
    ros::spin();


    return 0;
}