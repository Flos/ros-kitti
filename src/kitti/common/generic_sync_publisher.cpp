/*
 * sync.cpp
 *
 *  Created on: 23.01.2015
 *      Author: fnolden
 */

#include <common/generic_sync_publisher.h>

namespace kitti {

template <typename MessageT>
Generic_sync_publisher<MessageT>::Generic_sync_publisher(ros::NodeHandle nh, ros::NodeHandle nh_private) {
	calibration_exported = false;
	tf_exported = false;
	sequence = 0;
	this->nh_private = nh_private;
	this->nh = nh;
}

template <typename MessageT>
Generic_sync_publisher<MessageT>::~Generic_sync_publisher() {
	// TODO Auto-generated destructor stub
}

template <typename MessageT>
void
Generic_sync_publisher<MessageT>::init(){
	init_param();


	// init camera nodes;
	camera_nodes.resize(config_processd.image_topics.size());
	for(int i = 0; i < config_processd.image_topics.size(); ++i){
		camera_nodes.at(i).reset( new Camera_subscriber() );
		camera_nodes.at(i)->init(nh, config_processd.image_topics.at(i), config.data_destination, i, config.queue_size);
		camera_nodes.at(i)->create_image_info_sub(nh, config_processd.image_topics_info.at(i), config.queue_size);
	}

	// init pointcloud nodes
	pointcloud_nodes.resize(config_processd.pointcloud_topics.size());
	for(int i = 0; i < config_processd.pointcloud_topics.size(); ++i){
		std::stringstream folder_name;
		folder_name << "velodyne_points";
		if(i != 0){
			folder_name << i;
		}
		pointcloud_nodes.at(i).reset(new Pointcloud_subscriber());
		pointcloud_nodes.at(i)->init(nh, config_processd.pointcloud_topics.at(i), config.data_destination, folder_name.str(), config.queue_size);
	}

	// todo: init imu nodes
	//
	//

	// Set up dynamic reconfigure
	reconfigure_server.reset(new ReconfigureServer(nh));
	ReconfigureServer::CallbackType f = boost::bind(&Generic_sync_publisher::reconfigure_callback, this, _1, _2);
	reconfigure_server->setCallback(f);

	listener_transform.reset(new tf::TransformListener(nh, ros::Duration(config.tf_buffer_length), true));

	pub = nh.advertise<Sync_msg>(config.publish_topic, 1);
	sub = nh.subscribe<MessageT>(config.sync_topic, 1, boost::bind(&Generic_sync_publisher::callback, this, _1)  );
}

template <typename MessageT>
void
Generic_sync_publisher<MessageT>::init_param(){
	nh_private.getParam("subscribe_topics_image", config_processd.image_topics);
	nh_private.getParam("subscribe_topics_image_info", config_processd.image_topics_info);
	nh_private.getParam("camera_frame_ids", config_processd.image_frame_ids);
	nh_private.getParam("subscribe_topics_pcl", config_processd.pointcloud_topics);
	nh_private.getParam("pcl_frame_ids", config_processd.pointcloud_frame_ids);
	nh_private.getParam("publish", config.enabled);
	nh_private.getParam("sequence",config.sequence);

	assert(config_processd.image_topics.size() == config_processd.image_topics_info.size() );
	assert(config_processd.image_topics.size() > 0);
	assert(config_processd.image_frame_ids.size() <= config_processd.image_topics.size());

	assert(config_processd.pointcloud_topics.size() > 0);
	assert(config_processd.pointcloud_frame_ids.size() <= config_processd.pointcloud_topics.size());

	// Resize arrays to same size
	config_processd.image_topics.resize(config_processd.image_topics.size());
	config_processd.pointcloud_frame_ids.resize(config_processd.pointcloud_topics.size());
}

template <typename MessageT>
void
Generic_sync_publisher<MessageT>::tf_export(){

	assert(config_processd.image_frame_ids.size() == camera_nodes.size());

	bool all_valid = true;

	// Check if all camera nodes have received a calibration msg
	for(int i = 0; i < camera_nodes.size(); ++i){
		if(!camera_nodes.at(i)->calibration_valid){
			all_valid = false;
			return;
		}
	}

	// Set frame ids, look up and set transforms cam0_to_camX
	for(int i = 0; i < camera_nodes.size(); ++i){
		// if frame_id is provided use it instead
		if(!config_processd.image_frame_ids.at(i).empty()){
			camera_nodes.at(i)->calibration.frame_id = config_processd.image_frame_ids.at(i);
		}

		// if its not camera 0 look up transform from 0 to cam i
		tf::StampedTransform tf;
		if(i != 0){
			try{
			listener_transform->lookupTransform(
					camera_nodes.at(i)->calibration.frame_id.c_str(), /* target frame */
					camera_nodes.at(0)->calibration.frame_id.c_str(), /* source frame  */
					ros::Time::Time(0),	/* time */
					tf); /* transfrom */
			}catch (std::exception &e){
				ROS_WARN("Kitti export: transform look up failed: %s", e.what());
				return;
			}

			// set the transform
			camera_nodes.at(i)->calibration.tf.set_transform(tf);
			camera_nodes.at(i)->calibration.tf_rect.set_transform(tf);
		}
	}

	// save camera calibration
	camera_list_export();

	// Export calib_velo_to_cam0
	{
		tf::StampedTransform tf_velo_to_camera0;
		listener_transform->lookupTransform(
							camera_nodes.at(0)->calibration.frame_id.c_str(), /* target frame */
							pointcloud_nodes.at(0)->frame_id.c_str(), /* source frame  */
							ros::Time::Time(0),	/* time */
							tf_velo_to_camera0);

		kitti::filenames::save_tf_velo_to_camera0(config.data_destination, tf_velo_to_camera0);
	}

	// todo: Export calib_imu_to_velo
	{
//		for(int i = 0; i < imu_nodes.size(); ++i){
//			Tf calib_imu_to_velo;
//			tf::StampedTransform tf_velo_to_camera0;
//			listener_transform->lookupTransform(
//								imu_nodes.at(i).frame_id.c_str(), /* target frame */
//								pointcloud_nodes.at(0).frame_id.c_str(), /* source frame  */
//								ros::Time::Time(0),	/* time */
//								calib_imu_to_velo);
//
//			save.save_tf_velo_to_camera0(config.data_destination, tf_velo_to_camera0);
//		}
	}
	tf_exported = true;

}

template <typename MessageT>
void
Generic_sync_publisher<MessageT>::camera_list_export(){
	// Export camera calibrations calib_cam_to_cam.txt
	Camera_list cam_list;

	for(int i = 0; i < camera_nodes.size(); ++i){
		cam_list.cameras.push_back(camera_nodes.at(i)->calibration);
	}

	if( filenames::save_camera_list(config.data_destination, cam_list) ) {
		calibration_exported = true;
	}
	else{
		ROS_WARN("camera calibration export failed");
	}
}

template <typename MessageT>
void
Generic_sync_publisher<MessageT>::reconfigure_callback(Config &config, uint32_t level){
	this->config = config;
	return;
}

template <typename MessageT>
void
Generic_sync_publisher<MessageT>::callback(const MessageT &message){
	if(!config.enabled) return; //if publishing not enabled do nothing

	kitti::Sync_msg sync_msg;

	sync_msg.header.seq = sequence; 	++sequence;
	sync_msg.header.stamp = message.header.stamp;

	pub.publish(sync_msg);

	if(!tf_exported){
		tf_export();
	}
}

} /* namespace kitti */
