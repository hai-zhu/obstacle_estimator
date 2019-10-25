//
// Created by hai on 2/6/19.
//

#include <obstacle_estimator/obstacle_filter.h>

int main(int argc, char **argv)
{
    // Set up ROS
    ros::init(argc, argv, "obstacle_filter_node");  // node name
    ros::NodeHandle nh;                             // create a node handle

    // Initialize a class object and pass node handle for constructor
    Obstacle_Filter obstacle_filter(nh);

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