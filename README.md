# obstacle_estimator
Obstacle state estimation and prediction using ROS

#### Installation
```
$ cd [ROS_workspace]/src
$ git clone https://github.com/hai-zhu/obstacle_estimator.git
$ cd ..
$ catkin_make
```

#### Obstacle state estimation only
```
$ roslaunch obstacle_estimator obstacle_estimator_multiple.launch
```

#### Obstacle state estimation and prediction
```
$ roslaunch obstacle_estimator obstacle_prediction_multiple.launch
```
