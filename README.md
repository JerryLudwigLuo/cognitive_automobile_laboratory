# Cognitive_Automobile_Laboratory 2019 in MRT Institute, Karlsruher Institut für Technologie (KIT), Germany
In [this laboratory](https://www.mrt.kit.edu/lehre_SS_Kognitive_Automobile_Labor.php) we should write programs in C++ and Python and use ROS to implemente algorithms for autonomous driving.

During the compitition

![compitition video](https://github.com/JerryLudwigLuo/cognitive_automobile_laboratory/blob/master/anicar3_kal4/image/compitition.gif)

## Team NeverDrive

Perception part: Ossama Zenkri(traffic-sign detection), Jiawei Luo(pedestrian detection), Zhichong Xiao(obstecle detection), Xing Meng(pylon detection)

Control part: Jiawei Luo(lateral and longitudinal control) Xing Meng(path generation)

Our team won the champion of the compitition!

![champion](https://github.com/JerryLudwigLuo/cognitive_automobile_laboratory/blob/master/anicar3_kal4/image/champion.jpg)

## Lateral and Longitudinal Control

Basing on [stanley controller](http://ai.stanford.edu/~gabeh/papers/hoffmann_stanley_control07.pdf) we also added PI controller to let the car drive more smoothly. Because the car has no brake, we used cosine function to make car stop or slow down front of stop-sign, pedestrian(robot) or obstacles without shifting and sliding.

without PI controller

![without PI Controller](https://github.com/JerryLudwigLuo/cognitive_automobile_laboratory/blob/master/anicar3_kal4/image/without.gif)

with PI controller

![with PI Controller](https://github.com/JerryLudwigLuo/cognitive_automobile_laboratory/blob/master/anicar3_kal4/image/with.gif)

## Traffic-Sign Detection

We used color filter and 4 CNNs as classifier for 4 kinds of traffic-signs(stop, go straight, turn left and turn right).

e.p. stop sign detector

![Traffic Sign Detector](https://github.com/JerryLudwigLuo/cognitive_automobile_laboratory/blob/master/anicar3_kal4/image/sign.gif)

## Obstecle Detection

![avoiding box](https://github.com/JerryLudwigLuo/cognitive_automobile_laboratory/blob/master/anicar3_kal4/image/box.gif)

## Driving in unknown environment with pylons

with occupancy grid map and potential field to generate path in unknown environment. 

![grid map](https://github.com/JerryLudwigLuo/cognitive_automobile_laboratory/blob/master/anicar3_kal4/image/pylon.gif)

## Pedestrian(Robot) Detection

After some image-processing, we used Connected Components Label(CCL) to get segments, calculate HOG features of ROI, and use SVM as classifier to get measurement's value of the robot. In order to getting more accurate position of the robot, the car can track the robot with kalman filter.

robot doesn't across the road

![robot_pass](https://github.com/JerryLudwigLuo/cognitive_automobile_laboratory/blob/master/anicar3_kal4/image/robot1.gif)

robot is acrossing the road

![robot_stop](https://github.com/JerryLudwigLuo/cognitive_automobile_laboratory/blob/master/anicar3_kal4/image/robot2.gif)