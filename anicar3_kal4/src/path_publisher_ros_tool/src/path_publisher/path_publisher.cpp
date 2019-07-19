#include "path_publisher.hpp"
#include <tf2_eigen/tf2_eigen.h>
#include <cmath>
#include <boost/algorithm/clamp.hpp>
#include <boost/math/special_functions/sign.hpp>
#include <boost/range/algorithm/min_element.hpp>


namespace path_publisher_ros_tool {

PathPublisher::PathPublisher(ros::NodeHandle nhPublic, ros::NodeHandle nhPrivate)
        : interface_{nhPrivate}, reconfigureServer_{nhPrivate}, map_center_(7.0, 3.1),
          map_A_(8.5, 5.45), map_B_(4.56, 3.36), map_C_(6.3, 0.24), map_D_(9.72, 2.07),
          near_center_{false},pass_center_{false},sign_{0},current_task_{0} {

    /**
     * Initialization
     */
    interface_.fromParamServer();
    wanted_map_ = interface_.wanted_map;
    actual_map_ = interface_.actual_map;
    actual_map_index_ = interface_.actual_map -1;

    /**
     * Set up callbacks for subscribers and reconfigure.
     *
     * New subscribers can be created with "add_subscriber" in "cfg/PathPublisher.if file.
     * Don't forget to register your callbacks here!
     */

     wanted_map_publisher_ = nhPrivate.advertise<std_msgs::Int8>("/wanted_map", 1);

    set_path_subscriber_ = nhPrivate.subscribe("/DirectionSign",
    								        1,
    								        &PathPublisher::setSignCallback,
    								        this);                                           //

    detect_obstacle_subscriber_ = nhPrivate.subscribe("/obstacle_detection",
                                                       1,
                                                       &PathPublisher::detectObstacleCallback,
                                                       this);
    reconfigureServer_.setCallback(boost::bind(&PathPublisher::reconfigureRequest, this, _1, _2));

//  initial path_
    center_ = Eigen::Vector3d(interface_.center_x, interface_.center_y, 0.);             //default x=3,y=5
    readAllMaps(interface_.path_to_map);   //load maps to two-dimensional vector
    initialPartOfPath(actual_map_);

    path_publish_timer_ = nhPrivate.createTimer(ros::Rate(interface_.timer_rate), &PathPublisher::pathPublishCallback, this);
    switch_wanted_map_timer_ = nhPrivate.createTimer(ros::Rate(50), &PathPublisher::switchWantedMapCallback, this);
    task_publish_timer_  = nhPrivate.createTimer(ros::Rate(50), &PathPublisher::sendTaskCallback, this);

    rosinterface_handler::showNodeInfo();
}


void PathPublisher::setSignCallback(const std_msgs::Int8::ConstPtr& msg){
	 sign_ = msg->data;         //wanted_map = msg->data
	 //std::string s = msg->data;
}

void PathPublisher::detectObstacleCallback(const neverdrive_obstacle_detection_ros_tool::Box::Ptr msg){
    box_.box_state_ = msg->box_avoiding;
    box_.box_location_[0] = msg->box_location_x;
    box_.box_location_[1] = msg->box_location_y;
}

void PathPublisher::sendTaskCallback(const ros::TimerEvent & timerEvent){
    if(interface_.exit_active == 1){
        interface_.task_publisher.publish(current_task_);
    }
}

void PathPublisher::pathPublishCallback(const ros::TimerEvent & timerEvent) {
    if(current_task_ == 2){return;}
    int index_distance{0};    //how many points the car go through

    //get the vehicle position
    getVehiclePose();
    //shift vehicle position towards x direction kos_shift long
    const Eigen::Vector3d vehicle_position = vehicle_pose_.translation();
    Eigen::Vector3d vehicle_frame_unit_x = vehicle_pose_.rotation() * Eigen::Vector3d::UnitX();
    vehicle_frame_unit_x.z() = 0.0;
    vehicle_frame_unit_x = vehicle_frame_unit_x.normalized();
    const Eigen::Vector2d shifted_vehicle_position = (vehicle_position + vehicle_frame_unit_x * interface_.kos_shift).head<2>();


    //wanted_map_ = interface_.wanted_map;
    if(actual_map_ == wanted_map_){

        //only when the vehicle go enough distance, publish new path
        if(sample_flag_){
            //   find the closet point to vehicle on path (this moment)
            auto it = boost::range::min_element(
                    path_vector_, [&shifted_vehicle_position](const Eigen::Vector2d& le, const Eigen::Vector2d& re){
                        return (le - shifted_vehicle_position).squaredNorm() < (re - shifted_vehicle_position).squaredNorm();
                    });
            index_distance = std::distance(prev_pos_index_, it);             //how many points the car go through
            if (std::abs(index_distance * interface_.point_distance) < interface_.drive_distance) return;
        }else{
            //after initial or change path, publish the initial part_of_path_ first and set sample to true
            sample_flag_ = true;
            interface_.path_publisher.publish(part_of_path_);
            return;
        }

        //set new path to publish
        //clip a part of path to publish
        std::vector<Eigen::Vector2d>::iterator path_start;
        std::vector<Eigen::Vector2d>::iterator path_end;
        //reset prev pose whole index
        for(int i = 0; i < index_distance; i++){
            if (prev_pos_whole_index_ != all_path_vector_whole_[actual_map_index_].end() - 1)
                prev_pos_whole_index_++;
            else
                prev_pos_whole_index_ = all_path_vector_whole_[actual_map_index_].begin();
        }
        setCliper(prev_pos_whole_index_, all_path_vector_whole_[actual_map_index_], path_start, path_end);
        ROS_DEBUG_STREAM("pos index moving: " << index_distance << std::endl<<
                                              "current whole path index mark: " << std::distance(all_path_vector_whole_[actual_map_index_].begin(), prev_pos_whole_index_) << std::endl <<
                                              "current path start index: " << std::distance(all_path_vector_whole_[actual_map_index_].begin(), path_start) << std::endl <<
                                              "current path end index: " << std::distance(all_path_vector_whole_[actual_map_index_].begin(), path_end));
        clipPath(path_start, path_end, all_path_vector_whole_[actual_map_index_], path_vector_, part_of_path_);
        //		set new mark of prev pos index
        prev_pos_index_ = boost::range::min_element(
                path_vector_, [&shifted_vehicle_position](const Eigen::Vector2d& le, const Eigen::Vector2d& re){
                    return (le - shifted_vehicle_position).squaredNorm() < (re - shifted_vehicle_position).squaredNorm();
                });

        interface_.path_publisher.publish(part_of_path_);

        ROS_INFO_STREAM("actual map: " << actual_map_ << std::endl << "actual prev_pos_index point: " << *prev_pos_index_<< std::endl);
    }
    else{
        actual_map_ = wanted_map_;
        actual_map_index_ = actual_map_ - 1;
        initialPartOfPath(actual_map_);
        sample_flag_ = false;
    }


    ROS_INFO_STREAM("pathPublishCallback running");


}

void PathPublisher::readAllMaps(const std::string & mapPath) {
    std::vector<std::vector<Eigen::Vector2d>>::iterator iter = all_path_vector_whole_.begin();
    int count = 1;
    for(const auto& mapName: all_maps_name_){
        RoadMap map{49.01439, 8.41722};
        map.loadFromFile(mapPath + mapName + ".osm");
        double x, y;
        //save the first point
        map.getVertexMeters(1, 0, x, y);
        (*iter).emplace_back(Eigen::Vector2d(x, y));

        double accumulated_length = 0.;
        for(int i = 1; i < (int)map.trajectories.at(1).size(); i++){
            map.getVertexMeters(1, i, x, y);
            accumulated_length += ((*iter).back() - Eigen::Vector2d(x, y)).norm();
            if(accumulated_length > interface_.point_distance){
                (*iter).emplace_back(Eigen::Vector2d(x, y));
                accumulated_length = 0;
            }
        }

        ROS_INFO_STREAM("load map Nr." << count << " finished." << std::endl <<
                         "the whole path length: " << (*iter).size() << std::endl);
        iter++;
        count++;
    }
}

void PathPublisher::initialPartOfPath( const int actual_map ){

    actual_map_index_ = actual_map - 1;
    path_->header.frame_id = interface_.frame_id_map;
    path_->header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose_ros;
    pose_ros.pose.orientation.x = 0.0;
    pose_ros.pose.orientation.y = 0.0;
    pose_ros.pose.orientation.z = 0.0;
    pose_ros.pose.orientation.w = 0.0;
    pose_ros.pose.position.z = 0.0;
    pose_ros.header = path_->header;

    getVehiclePose();           //save vehicle pose to vehicle_pose_

    //set prev_pos_index mark
    const Eigen::Vector3d vehicle_position = vehicle_pose_.translation();
    Eigen::Vector3d vehicle_frame_unit_x = vehicle_pose_.rotation() * Eigen::Vector3d::UnitX();
    vehicle_frame_unit_x.z() = 0.0;
    vehicle_frame_unit_x = vehicle_frame_unit_x.normalized();
    const Eigen::Vector2d pos2d = (vehicle_position + vehicle_frame_unit_x * interface_.kos_shift).head<2>();   //2d pose of the center of the car

    //find the closet point(on path) to center of vehicle
   if(actual_map_ == 9 || actual_map_ ==10 ){ prev_pos_whole_index_ = box_closest_wanted_map_iter_;}else {
    prev_pos_whole_index_ = boost::range::min_element(
            all_path_vector_whole_[actual_map_index_], [&pos2d](const Eigen::Vector2d& le, const Eigen::Vector2d& re){
                return (le - pos2d).squaredNorm() < (re - pos2d).squaredNorm();
            });
   }

    std::vector<Eigen::Vector2d>::iterator path_start, path_end;
    setCliper(prev_pos_whole_index_, all_path_vector_whole_[actual_map_index_], path_start, path_end);
    clipPath(path_start, path_end, all_path_vector_whole_[actual_map_index_], path_vector_, part_of_path_);             //give value to path_vector and part_of_path_

    ROS_DEBUG_STREAM("initial whole path length: " << all_path_vector_whole_[actual_map_index_].size() << std::endl <<
                                           "initial part of path length: " << path_vector_.size() << std::endl <<
                                           "initial path message length: " << part_of_path_->poses.size());

    prev_pos_index_ = boost::range::min_element(
            path_vector_, [&pos2d](const Eigen::Vector2d& le, const Eigen::Vector2d& re){
                return (le - pos2d).squaredNorm() < (re - pos2d).squaredNorm();
            });
}


void PathPublisher::getVehiclePose() {
    while(1){
        try {
            const geometry_msgs::TransformStamped tf_ros =
                    tfBuffer_.lookupTransform(interface_.frame_id_map, interface_.frame_id_vehicle, ros::Time(0));
            vehicle_pose_ = tf2::transformToEigen(tf_ros);
            break;
        } catch (const tf2::TransformException& e){
            ROS_WARN_STREAM(e.what());
        }
    }


}

double signedAngleBetween(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    const double vz = boost::math::sign(a.cross(b).z());
    return vz * std::acos(a.normalized().dot(b.normalized()));
}

void PathPublisher::clipPath(std::vector<Eigen::Vector2d>::iterator& source_start,
                             std::vector<Eigen::Vector2d>::iterator& source_end,
                             std::vector<Eigen::Vector2d>& source,
                             std::vector<Eigen::Vector2d>& dest,
                             nav_msgs::Path::Ptr& path_ptr){
    //initial pose message
    geometry_msgs::PoseStamped pose_ros;
    pose_ros.pose.orientation.x = 0.0;
    pose_ros.pose.orientation.y = 0.0;
    pose_ros.pose.orientation.z = 0.0;
    pose_ros.pose.orientation.w = 0.0;
    pose_ros.pose.position.z = 0.0;
    pose_ros.header = path_->header;
    dest.clear();
    path_ptr.reset(new nav_msgs::Path);
    path_ptr->header = path_->header;
    for(auto ele = source_start; ele != source_end;){
        dest.emplace_back(*ele);
        pose_ros.pose.position.x = (*ele)[0];
        pose_ros.pose.position.y = (*ele)[1];
        path_ptr->poses.emplace_back(pose_ros);
        if (ele == source.end() - 1){
            ele = source.begin();
        }else ele++;
    }
}


void PathPublisher::setCliper(std::vector<Eigen::Vector2d>::iterator& it, std::vector<Eigen::Vector2d>& source, std::vector<Eigen::Vector2d>::iterator& it_start, std::vector<Eigen::Vector2d>::iterator& it_end){
//		clip a part of path to publish
    auto path_start = it;
    int index_dis = 1;
    if(path_start == source.begin())
    {
        path_start = source.end()-1;
        index_dis++;
    }
//		find the beginning of this part first
    for(--path_start; path_start != it;){
        if ((index_dis * interface_.point_distance) > interface_.path_length/4.0)
            break;
//			link end to start
        if (path_start != source.begin()){
            path_start--;
            index_dis++;
        }else{
            path_start = source.end() - 1;
            index_dis++;
        }
    }
    it_start = path_start;
//		find the end of this part
    auto path_end = it;
    index_dis = 1;
    if(path_end == source.end()-1)
    {
        path_end = source.begin();
        index_dis++;
    }
    for(++path_end; path_end != it;){
        if((index_dis * interface_.point_distance) > 3.0*interface_.path_length/4.0)
            break;
//			link end to start
        if (path_end != source.end() - 1){
            path_end++;
            index_dis++;
        }else{
            index_dis++;
            path_end = source.begin();
        }
    }
    it_end = path_end;
}


/*******Functions relative to switch the maps (use which logic)***********/
bool PathPublisher::nearCenter() {
    getVehiclePose();
    const Eigen::Vector2d vehcile_position = vehicle_pose_.translation().head<2>();
    //return  (vehcile_position - map_center_).squaredNorm() < interface_.near_center_distance;
    return (vehcile_position[0] > map_center_[0]- interface_.near_center_distance && vehcile_position[0] < map_center_[0] + interface_.near_center_distance );

}

int PathPublisher::nearTurnPoint() {
    getVehiclePose();
    const Eigen::Vector2d vector_position = vehicle_pose_.translation().head<2>();
    std::vector<double> distance;
    distance.push_back((vector_position - map_A_).squaredNorm());
    distance.push_back((vector_position - map_B_).squaredNorm());
    distance.push_back((vector_position - map_C_).squaredNorm());
    distance.push_back((vector_position - map_D_).squaredNorm());
    auto minIter = std::min_element(distance.begin(), distance.end());

    ROS_DEBUG_STREAM("nearst A(0) B(1) C(2) D(3) is: " << (minIter - distance.begin()) );

    return (minIter - distance.begin());
}


void PathPublisher::switchWantedMapCallback(const ros::TimerEvent& timerEvent){

    if(interface_.robot_task == 1){ wanted_map_ = 5; return;}

    ROS_DEBUG_STREAM("nearst point "<< nearTurnPoint());
	ROS_DEBUG_STREAM("near Center"<< nearCenter());
    //ROS_DEBUG_STREAM("wanted map"<< wanted_map_);
    wanted_map_publisher_.publish(actual_map_);
    //sign_ = interface_.sign;

    if(current_task_==3){return;}
    if(interface_.exit_active == 1){
        if( current_task_ == 2) {
		if(vehicle_pose_.translation()[0] > 11.0){
			current_task_ =3;}
		return;}

        if(inExitArea() && sign_==1 && (actual_map_ == 1 || actual_map_ == 3 || actual_map_ == 5 ) ){wanted_map_ =11;     current_task_=2;  return;}
    }

    /*************change path according to obstacle*************/

    if(box_.box_state_ == "box_exist" && boxToPathDist(box_.box_location_, path_vector_) < interface_.dist_to_path && actual_map_ != 9 && actual_map_ !=10){


        //up area
        if(nearTurnPoint()==1 || nearTurnPoint()==2){
            //outside
            if(actual_map_==2 || actual_map_==3 || actual_map_==7){
                wanted_map_ = 9;
                back_to_map_=2;
               // obstacle_avoiding_time_=ros::Time::now();
            }
            else if(actual_map_==1 || actual_map_==6 || actual_map_==4){   //inside
                wanted_map_ = 10;
                back_to_map_=1;
               // obstacle_avoiding_time_=ros::Time::now();
		ROS_DEBUG_STREAM("up change from 1 to 10"<< wanted_map_);
            }

        }else if(nearTurnPoint()==0 || nearTurnPoint()==3){      //down area
            //outside
            if(actual_map_==1 || actual_map_==3 || actual_map_==5){
                wanted_map_ = 10;
                back_to_map_ = 1;
               // obstacle_avoiding_time_=ros::Time::now();
            }
            else if(actual_map_==2 || actual_map_==4 || actual_map_==8){   //inside
                wanted_map_ = 9;
                back_to_map_=2;
                //obstacle_avoiding_time_=ros::Time::now();
            }
        }

        getVehiclePose();
        Eigen::Vector2d vehicle_location =  vehicle_pose_.translation().head<2>();
        std::vector<Eigen::Vector2d>::iterator car_closest_wanted_map_iter = findClosestIter(vehicle_location, wanted_map_);
        box_closest_wanted_map_iter_ = findClosestIter(box_.box_location_, wanted_map_);
        if(safeIterDistance(car_closest_wanted_map_iter, box_closest_wanted_map_iter_, wanted_map_) < 60 ){
            //find closed point to back_to_map
            start_avoiding_iter_ = findClosestIter(box_.box_location_, back_to_map_);
        }else{
            wanted_map_ = actual_map_;

            back_to_map_ = actual_map_;
        }

    }

    if(actual_map_ == 9 || actual_map_ == 10){ current_task_ = 1; }else{ current_task_=0;}


    if(actual_map_ == 9 || actual_map_ ==10){
        getVehiclePose();
        Eigen::Vector2d vehicle_location =  vehicle_pose_.translation().head<2>();
        std::vector<Eigen::Vector2d>::iterator current_iter = findClosestIter(vehicle_location, back_to_map_);
        //safe iter distance
        int iterDist = safeIterDistance(start_avoiding_iter_, current_iter, back_to_map_);
        if(  iterDist > interface_.go_back_iter_dist &&  iterDist < 120){
            wanted_map_=back_to_map_;
        }
        return;
    }


    /*********change path according to traffic sign***********/
    if(nearCenter() && sign_!=0 && (actual_map_==1 || actual_map_==2)){
        switch(nearTurnPoint()){
            case 0: //A
                if(sign_==-1){ wanted_map_=5;}
                if(sign_==1){ wanted_map_=3;}
		        break;
            case 1: //B
                if(sign_==-1){ wanted_map_=4;}
                if(sign_==1){ wanted_map_=6;}
		        break;
            case 2: //C
                if(sign_==-1){ wanted_map_=7;}
                if(sign_==1){ wanted_map_=3;}
		        break;
            case 3: //D
                if(sign_==-1){ wanted_map_=4;}
                if(sign_==1){ wanted_map_=8;}
            default: break;
        }

    }

    if(!nearCenter() && actual_map_!=1 && actual_map_!=2){
        switch(nearTurnPoint()){
            case 0: //A
                wanted_map_=2;
		        break;
            case 1: //B
                wanted_map_=2;
		        break;
            case 2: //C
                wanted_map_=1;
		        break;
            case 3: //D
                wanted_map_=1;
            default: break;
        }
    }


}

bool PathPublisher::inExitArea(){
    getVehiclePose();
    const Eigen::Vector2d vehcile_position = vehicle_pose_.translation().head<2>();

    return (vehcile_position[0] > interface_.exit_area_x_min &&
            vehcile_position[0] < interface_.exit_area_x_max &&
            vehcile_position[1] > interface_.exit_area_y_min &&
            vehcile_position[1] < interface_.exit_area_y_max);
}

double PathPublisher::boxToPathDist(Eigen::Vector2d& boxLocation,std::vector<Eigen::Vector2d>& Path) {
    std::vector<Eigen::Vector2d>::iterator it = boost::range::min_element(
            Path, [&boxLocation](const Eigen::Vector2d &le, const Eigen::Vector2d &re) {
                return (le - boxLocation).squaredNorm() < (re - boxLocation).squaredNorm();
            });

    const Eigen::Vector2d closed_point = *it;

    const double dist = (boxLocation - closed_point).norm();
    ROS_INFO_STREAM("distance from box to path: "<< dist);
    return dist;
}

std::vector<Eigen::Vector2d>::iterator PathPublisher::findClosestIter(Eigen::Vector2d& point, int mapNum){

    int index = mapNum - 1;
    std::vector<Eigen::Vector2d>::iterator it = boost::range::min_element(
            all_path_vector_whole_[index], [&point](const Eigen::Vector2d &le, const Eigen::Vector2d &re) {
                return (le - point).squaredNorm() < (re - point).squaredNorm();
            });
    return it;
}

int PathPublisher::safeIterDistance(std::vector<Eigen::Vector2d>::iterator& iter_start, std::vector<Eigen::Vector2d>::iterator& iter_end, int mapNum){
    int index = mapNum-1;
    if( std::distance(iter_start,iter_end) >= 0) {
        return std::distance(iter_start,iter_end);
    }else {
        return std::distance(iter_start, all_path_vector_whole_[index].end()) + std::distance(all_path_vector_whole_[index].begin(), iter_end);
    }
}

/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
*/
void PathPublisher::reconfigureRequest(const Interface::Config& config, uint32_t level) {
    interface_.fromConfig(config);
}


} // namespace path_publisher_ros_tool
