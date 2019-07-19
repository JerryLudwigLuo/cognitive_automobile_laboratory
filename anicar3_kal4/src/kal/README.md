# Kognitive Automotive Lab 

This is a meta-packge to provide easy access to all KAL-packages.

Additionaly it provied the master.launch file, which starts all given componenets. If you want to get to know the system, this is a good staring point.

## start

when you enter the workspace, don't forget source the setup.bash,

simple run `mrtsrc`
## use joystick to control anicar
1. `export CAR_NAME=anicar3` or `export CAR_NAME=anicar2`
   it depends on which anicar you utilize
  
2. `roslaunch kal master_live.launch`

3. after Rviz window shows, press the A button of joystick to activate motor

## Generate path map

1. use `rosbag record /tf /tf_static /landmarks /trajectory` to generate rosbag when master_live.launch is runnung

2. use path_generator_ros_tool package to generate map

## path following

run `roslaunch kal demo_path_follower.launch`


## License

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
