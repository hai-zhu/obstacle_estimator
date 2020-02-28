//
// Created by hai on 22/4/19.
//

#ifndef OBSTACLE_ESTIMATOR_OBSTACLE_PREDICTION_H
#define OBSTACLE_ESTIMATOR_OBSTACLE_PREDICTION_H

// General include
#include <math.h>
#include <time.h>
#include <vector>

// Eigen includes
#include <Eigen/Dense>

// Ros include
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

// Message types
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

// Custom message includes. Auto-generated from msg/ directory.


// Define a class, including a constructor, member variables and member functions
class Obstacle_Prediction
{
public:
    //! Constructor, "main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    explicit Obstacle_Prediction(ros::NodeHandle nh, double delta_t, int horizon_N);

private:
    //! Ros node handle
    ros::NodeHandle     nh_;        // we will need this, to pass between "main" and constructor

    //! Some objects to support subscriber, service, and publisher
    ros::Subscriber     sub_;
    ros::Publisher      pub_;
    ros::Publisher      odo_pub_;

    //! Obstacle measurement
    Eigen::Vector3d     pos_measured_;          // measured position information

    //! Time information for filter
    ros::Time           time_stamp_;            // time stamp of current measurement
    ros::Time           time_stamp_previous_;   // time stamp of last measurement
    double              dt_;                    // time difference between two measurements

    //! Time information for predictor
    double              delta_t_;               // delta t of the predicted horizon
    int                 horizon_N_;             // prediction horizon length             

    //! Obstacle estimation and prediction
    Eigen::Matrix<double, 6, 1> state_estimated_;       // estimated state (pos & vel)
    Eigen::Matrix<double, 6, 6> state_cov_estimated_;   // estimated covariance matrix

    //! Initializations
    void initializeSubscribers();
    void initializePublishers();

    //! Subscriber callback
    void subscriberCallback(const geometry_msgs::PoseStamped &msg);

};



#endif //OBSTACLE_ESTIMATOR_OBSTACLE_FILTER_H
