//
// Created by hai on 2/4/19.
//


#include <obstacle_estimator/obstacle_filter.h>

// Constructor:  this will get called whenever an instance of this class is created
Obstacle_Filter::Obstacle_Filter(ros::NodeHandle nh): nh_(nh)
{
    ROS_INFO("In class constructor of Obstacle_Filter");

    // Initialization subscriber and publisher
    initializeSubscribers();
    initializePublishers();

    // Initialization the state covariance, this is important for starting the Kalman filter
    double cov_pos = 1^2;
    double cov_vel = 1^2;
    state_cov_estimated_.setZero();
    state_cov_estimated_(0, 0) = cov_pos;
    state_cov_estimated_(1, 1) = cov_pos;
    state_cov_estimated_(2, 2) = cov_pos;
    state_cov_estimated_(3, 3) = cov_vel;
    state_cov_estimated_(4, 4) = cov_vel;
    state_cov_estimated_(5, 5) = cov_vel;

    // Other initialization
    pos_measured_.setZero();
    state_estimated_.setZero();
    
    time_stamp_ = ros::Time::now();
    time_stamp_previous_ = ros::Time::now();
    dt_ = 0.001;

}


// Set up subscribers
void Obstacle_Filter::initializeSubscribers()
{
    ROS_INFO("Initializing subscribers");
    sub_ = nh_.subscribe("/obstacle/pose", 1, &Obstacle_Filter::subscriberCallback, this);
}


// Set up publisher
void Obstacle_Filter::initializePublishers()
{
    ROS_INFO("Initializing publishers");
    pub_ = nh_.advertise<nav_msgs::Odometry>("/obstacle/state_estimation", 1, true);
}


// Subscriber callback function
void Obstacle_Filter::subscriberCallback(const geometry_msgs::PoseStamped &msg)
{
    // the real work is done in this callback function
    // it wakes up every time a new message is published on obstacle_sub_topic_

    // for debugging
    // ROS_INFO("Filtering");

    // get measured position
    pos_measured_(0) = msg.pose.position.x;
    pos_measured_(1) = msg.pose.position.y;
    pos_measured_(2) = msg.pose.position.z;

    // current time stamp of the message
    time_stamp_      = msg.header.stamp;

    // time difference. If using the node_rate to derive, then comment the following lines
    dt_ = (time_stamp_ - time_stamp_previous_).toSec();   // sec

    // perform Kalman Filtering
    // matrix of state transition model
    Eigen::Matrix<double, 6, 6> A;
    A << 1, 0, 0, dt_, 0, 0,
         0, 1, 0, 0, dt_, 0,
         0, 0, 1, 0, 0, dt_,
         0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1;
    Eigen::Matrix<double, 6, 3> B;
    double dt_2 = 0.5*dt_*dt_;
    B << dt_2, 0, 0,
         0, dt_2, 0,
         0, 0, dt_2,
         0, 0, 0,
         0, 0, 0,
         0, 0, 0;

    // matrix of observation model
    Eigen::Matrix<double, 3, 6> H;
    H << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0;

    // noise matrix
    double R_pos = 10E-4;
    Eigen::Matrix<double, 3, 3> R;      // observation noise covariance
    R << R_pos, 0, 0,
         0, R_pos, 0,
         0, 0, R_pos;
    double Q_acc = 1000;                // mean of noisy acceleration
    double Q_pos = Q_acc * dt_2;
    double Q_vel = Q_acc * dt_;
    Eigen::Matrix<double, 6, 6> Q;
    Q << Q_pos, 0, 0, 0, 0, 0,
         0, Q_pos, 0, 0, 0, 0,
         0, 0, Q_pos, 0, 0, 0,
         0, 0, 0, Q_vel, 0, 0,
         0, 0, 0, 0, Q_vel, 0,
         0, 0, 0, 0, 0, Q_vel;

    // prediction
    Eigen::Vector3d u(0, 0, 0);
    state_estimated_ = A*state_estimated_ + B*u;
    state_cov_estimated_ = A*state_cov_estimated_*A.transpose() + Q;

    // update
    Eigen::Vector3d pos_residual;       // pos estimation residual
    pos_residual = pos_measured_ - H * state_estimated_;
    Eigen::Matrix<double, 3, 3> S;
    S = H*state_cov_estimated_*H.transpose() + R;
    Eigen::Matrix<double, 6, 3> K;      // the Kalman gain matrix
    K = (state_cov_estimated_*H.transpose()) * S.inverse();
    // new estimated state
    state_estimated_ = state_estimated_ + K*pos_residual;
    // new covariance
    Eigen::Matrix<double, 6, 6> I;
    I.setIdentity();                    // I is an identity matrix
    state_cov_estimated_ = (I - K*H) * state_cov_estimated_;
    // save covariance as an array, cov is symmetric, thus its row and column major representation are the same
    Eigen::Map<Eigen::RowVectorXd> state_cov_vector(state_cov_estimated_.data(), state_cov_estimated_.size());

    // prepare published message
    nav_msgs::Odometry msg_pub;                         // published obstacle state estimation
    msg_pub.header = msg.header;                        // save the header information
    msg_pub.pose.pose.orientation = msg.pose.orientation;
    msg_pub.pose.pose.position.x = state_estimated_(0); // save the estimated position
    msg_pub.pose.pose.position.y = state_estimated_(1);
    msg_pub.pose.pose.position.z = state_estimated_(2);
    msg_pub.twist.twist.linear.x = state_estimated_(3); // save the estimated velocity
    msg_pub.twist.twist.linear.y = state_estimated_(4);
    msg_pub.twist.twist.linear.z = state_estimated_(5);

    // save the estimated position and velocity covariance
    // position covariance
    msg_pub.pose.covariance[0] = state_cov_estimated_(0,0);
    msg_pub.pose.covariance[1] = state_cov_estimated_(0,1);
    msg_pub.pose.covariance[2] = state_cov_estimated_(0,2);
    msg_pub.pose.covariance[6] = state_cov_estimated_(1,0);
    msg_pub.pose.covariance[7] = state_cov_estimated_(1,1);
    msg_pub.pose.covariance[8] = state_cov_estimated_(1,2);
    msg_pub.pose.covariance[12] = state_cov_estimated_(2,0);
    msg_pub.pose.covariance[13] = state_cov_estimated_(2,1);
    msg_pub.pose.covariance[14] = state_cov_estimated_(2,2);
    // velocity covariance
    msg_pub.twist.covariance[0] = state_cov_estimated_(3,3);
    msg_pub.twist.covariance[1] = state_cov_estimated_(3,4);
    msg_pub.twist.covariance[2] = state_cov_estimated_(3,5);
    msg_pub.twist.covariance[6] = state_cov_estimated_(4,3);
    msg_pub.twist.covariance[7] = state_cov_estimated_(4,4);
    msg_pub.twist.covariance[8] = state_cov_estimated_(4,5);
    msg_pub.twist.covariance[12] = state_cov_estimated_(5,3);
    msg_pub.twist.covariance[13] = state_cov_estimated_(5,4);
    msg_pub.twist.covariance[14] = state_cov_estimated_(5,5);

    // publish the message
    pub_.publish(msg_pub);

    // set time
    time_stamp_previous_ = time_stamp_;
}
