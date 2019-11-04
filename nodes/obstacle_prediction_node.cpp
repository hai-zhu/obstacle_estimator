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
    double delta_t;                                     // delta t for prediction
    if (!nh.getParam(ros::this_node::getName()+"/delta_t", delta_t))
    {
        ROS_ERROR_STREAM("obstacle_prediction_node Parameter " << ros::this_node::getName()+"/delta_t not set");
        return 0;
    }
//    delta_t = 0.6;     // for debugging

    int horizon_N;                                      // prediction horizon length
    if (!nh.getParam(ros::this_node::getName()+"/horizon_N", horizon_N))
    {
        ROS_ERROR_STREAM("obstacle_prediction_node Parameter " << ros::this_node::getName()+"/horizon_N not set");
        return 0;
    }
//    horizon_N = 20;     // for debugging

    // Initialize a class object and pass node handle for constructor
    Obstacle_Prediction obstacle_prediction(nh, delta_t, horizon_N);

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