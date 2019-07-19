#include "path_generator.hpp"
#include "bag.hpp"
#include <tf/tfMessage.h>
#include <eigen_conversions/eigen_msg.h>
#include "road_map/RoadMap.hpp"
#include "cornucopia/Cornucopia.h"

namespace path_generator_ros_tool {

PathGenerator::PathGenerator(ros::NodeHandle nhPublic, ros::NodeHandle nhPrivate)
        : interface_{nhPrivate}, reconfigureServer_{nhPrivate} {

    /**
     * Initialization
     */
    interface_.fromParamServer();


    /**
     * Set up callbacks for subscribers and reconfigure.
     *
     * New subscribers can be created with "add_subscriber" in "cfg/PathGenerator.if file.
     * Don't forget to register your callbacks here!
     */
    reconfigureServer_.setCallback(boost::bind(&PathGenerator::reconfigureRequest, this, _1, _2));

//get sample points from rosbag file
    const std::vector<tf::tfMessage::ConstPtr> msgs_tfs = fromBag<tf::tfMessage>(interface_.bag_file_name, "/tf");
    const std::vector<tf::tfMessage::ConstPtr> msgs_static_tfs = fromBag<tf::tfMessage>(interface_.bag_file_name, "/tf_static");
//    ROS_DEBUG_STREAM("Found " << msgs_tfs.size() << " msgs in /tf.");
//    ROS_DEBUG_STREAM("Found " << msgs_static_tfs.size() << " msgs in /tf_static.");
    bool found_stargazer_to_map = false, found_vehicle_to_camera = false;
    Eigen::Affine3d tf_stargazer_to_map_eigen, tf_vehicle_to_camera_eigen;
    for (auto const& msg_tfs: msgs_static_tfs){
    	for (auto const& tf: msg_tfs->transforms){
//    		ROS_DEBUG_STREAM("a header frame_id: " << tf.header.frame_id << "\t" << "and child frame i: " << tf.child_frame_id);
    		if (tf.header.frame_id == interface_.frame_id_stargazer && tf.child_frame_id == interface_.frame_id_map){
    			tf::transformMsgToEigen(tf.transform, tf_stargazer_to_map_eigen);
    			found_stargazer_to_map = true;
    		}else if (tf.header.frame_id == interface_.frame_id_vehicle && tf.child_frame_id == interface_.frame_id_camera){
    			tf::transformMsgToEigen(tf.transform, tf_vehicle_to_camera_eigen);
    			found_vehicle_to_camera =true;
    		}else if (tf.header.frame_id == interface_.frame_id_map && tf.child_frame_id == interface_.frame_id_stargazer){
    			tf::transformMsgToEigen(tf.transform, tf_stargazer_to_map_eigen);
    			tf_stargazer_to_map_eigen = tf_stargazer_to_map_eigen.inverse();
    			found_stargazer_to_map = true;
    		}else if (tf.header.frame_id == interface_.frame_id_camera && tf.child_frame_id == interface_.frame_id_vehicle){
    			tf::transformMsgToEigen(tf.transform, tf_vehicle_to_camera_eigen);
    			tf_vehicle_to_camera_eigen = tf_vehicle_to_camera_eigen.inverse();
    			found_vehicle_to_camera =true;
    		}
    		if (found_stargazer_to_map && found_vehicle_to_camera)
    			break;
     	}
    }

    ROS_DEBUG_STREAM("start to parse poses");
	for (auto const& msg_tfs: msgs_tfs){
		Eigen::Affine3d tf_camera_to_stargazer_eigen;
		for (auto const& tf: msg_tfs->transforms){
			if (tf.header.frame_id == interface_.frame_id_stargazer && tf.child_frame_id == interface_.frame_id_camera)
					tf::transformMsgToEigen(tf.transform, tf_camera_to_stargazer_eigen);
			else if (tf.header.frame_id == interface_.frame_id_camera && tf.child_frame_id == interface_.frame_id_stargazer){
				tf::transformMsgToEigen(tf.transform, tf_camera_to_stargazer_eigen);
				tf_camera_to_stargazer_eigen = tf_camera_to_stargazer_eigen.inverse();
			} else
				continue;
			const Eigen::Affine3d tf_vehicle_to_map_eigen{tf_stargazer_to_map_eigen * tf_camera_to_stargazer_eigen *
									tf_vehicle_to_camera_eigen.inverse()};
			trajectory_.emplace_back(tf_vehicle_to_map_eigen);
		}
	}
	ROS_DEBUG_STREAM("Found " << trajectory_.size() << " poses in rosbag.");

    homogenizePoints();
    ROS_DEBUG_STREAM(trajectory_.size() << " homogenous poses remain.");
//	save the sampled raw map
    toRoadMap("raw_path.osm");
//	fit map and store it
    mapFitting("fitted_road_map.osm");

    rosinterface_handler::showNodeInfo();
}

void PathGenerator::homogenizePoints(){
	Eigen::Affine3d previous_pose = trajectory_.front();
	Eigen::Vector3d this_pose;
	std::vector<Eigen::Affine3d> poses_list;
	for (auto const& pose: trajectory_){
		this_pose = pose.translation();
//		ROS_DEBUG_STREAM("Position: " << this_pose[0] << "\t" << this_pose[1] << "\t" << this_pose[2]);
		Eigen::Vector2d d_vector = previous_pose.translation().head<2>() - pose.translation().head<2>();
//		ROS_DEBUG_STREAM("distance of two near points: " << d_vector.norm());
		if (d_vector.norm() > interface_.input_distance){
			poses_list.emplace_back(pose);
			previous_pose = pose;
		}
	}
	trajectory_ = poses_list;
}

void PathGenerator::toRoadMap(std::string filename){
	RoadMap roadMap(49.01439, 8.41722);
	roadMap.newTrajectory(1);
	for (const auto& pose: trajectory_){
		Eigen::Vector3d pose3d = pose.translation();
		roadMap.appendVertexMeters(pose3d[0], pose3d[1], 1);
	}
	std::string path = interface_.path_to_map + filename;
	roadMap.saveToFile(path);
}

void PathGenerator::mapFitting(std::string filename){
	RoadMap map(interface_.path_to_map + "raw_path.osm", 49.01439, 8.41722);
	Cornu::Fitter fitter;
//	initialize the fitter
	Cornu::Parameters params(Cornu::Parameters::ACCURATE);
	params.set(Cornu::Parameters::LINE_COST, 7.5);
	params.set(Cornu::Parameters::ARC_COST, 9.);
	params.set(Cornu::Parameters::CLOTHOID_COST, 15.);
	params.set(Cornu::Parameters::G0_COST, Cornu::Parameters::infinity);
	params.set(Cornu::Parameters::G1_COST, 1000.);
	params.set(Cornu::Parameters::G2_COST, 0.);
	params.set(Cornu::Parameters::ERROR_COST, interface_.error_cost); //500
	params.set(Cornu::Parameters::SHORTNESS_COST, 0.);
	params.set(Cornu::Parameters::INFLECTION_COST, 0.);
	params.set(Cornu::Parameters::CLOSEDNESS_THRESHOLD, 1);
	params.set(Cornu::Parameters::POINTS_PER_CIRCLE, interface_.points_per_circle); //100
//	params.set(Cornu::Parameters::CURVE_ADJUST_DAMPING, 1000.);
	params.set(Cornu::Parameters::CURVATURE_ESTIMATE_REGION, interface_.curvature_estimate_region);
//	params.set(Cornu::Parameters::OVERSKETCH_THRESHOLD, 1.);
	fitter.setParams(params);
	Cornu::VectorC<Eigen::Vector2d> points(map.numberOfVertices(1), Cornu::CIRCULAR);
//	ROS_DEBUG_STREAM("initial a points list of " << map.numberOfVertices(1) << " size." << std::endl <<
//					"start to save vertex");
	double x, y;
	for (int i = 0; i < (int)map.trajectories.at(1).size(); i++){
		map.getVertexMeters(1, i, x, y);
		points[i] = Eigen::Vector2d(x, y);
	}

//		pass it to the fitter and process it
	fitter.setOriginalSketch(new Cornu::Polyline(points));
	fitter.run();
// process the output -- count the number of primitives of each type
	Cornu::PrimitiveSequenceConstPtr output = fitter.finalOutput();
	int nPrims[3] = {0, 0, 0};
	for (int i = 0; i < output->primitives().size(); ++i) {
		nPrims[output->primitives()[i]->getType()]++;
	}
	ROS_DEBUG_STREAM("Fitting finished, #lines = " << nPrims[0] << ", #arcs = " << nPrims[1]
					<< ", #clothoids = " << nPrims[2]);

//	save fitted path
	RoadMap map_out(49.01439, 8.41722);
	map_out.newTrajectory(1);
	Eigen::Vector2d fitted_point;
	ROS_DEBUG_STREAM("Start to path " << output->length() << " long" << std::endl <<
					"output_distance: " << interface_.output_distance);
	int count{0};
	for (double s = 0; s < output->length(); s += interface_.output_distance) {
		output->eval(s, &fitted_point);
		count++;
		map_out.appendVertexMeters(fitted_point.x(), fitted_point.y(), 1);
	}
	map_out.saveToFile(interface_.path_to_map + filename);

}

/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
*/
void PathGenerator::reconfigureRequest(const Interface::Config& config, uint32_t level) {
    interface_.fromConfig(config);
}


} // namespace path_generator_ros_tool
