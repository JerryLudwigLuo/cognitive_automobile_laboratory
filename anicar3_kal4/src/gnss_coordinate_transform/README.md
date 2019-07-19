# GNSS COORDINATE TRANSFORM

* wrapper for geographiclib
* previously contained local/MRT meractor transformation library; replaced by UTM
  * currently issues compiler warnings for that
* by first setting an origin, it checks whether the UTM zone is left and throws an error if so
* for using it with ROS, see [gnss_coordinate_transform_ros](https://gitlab.mrt.uni-karlsruhe.de/MRT/gnss_coordinate_transform_ros/)
