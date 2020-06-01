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

If you find this code useful in your research then please cite:
```
@article{Zhu2019RAL,
    title = {{Chance-Constrained Collision Avoidance for MAVs in Dynamic Environments}},
    author = {Zhu, Hai and Alonso-Mora, Javier},
    journal = {IEEE Robotics and Automation Letters},
    number = {2},
    volume = {4},
    pages = {776--783},
    publisher = {IEEE},
    year = {2019}
}
```
