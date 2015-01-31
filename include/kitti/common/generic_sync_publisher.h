/*
 * sync.h
 *
 *  Created on: 23.01.2015
 *      Author: fnolden
 */

#include <dynamic_reconfigure/server.h>
#include <kitti/kitti_exportConfig.h>
#include <kitti/Sync_msg.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>

#include <kitti/common/serialization/filenames.h>
#include <kitti/common/serialization/camera.h>
#include <kitti/common/camera_subscriber.h>
#include <kitti/common/pointcloud_subscriber.h>
#include <kitti/common/generic_subscriber.h>

#include <boost/bind/protect.hpp>
#include <boost/ref.hpp>


#ifndef SRC_KITTI_EXPORT_COMMON_SYNC_H_
#define SRC_KITTI_EXPORT_COMMON_SYNC_H_

namespace kitti {

struct Config_list{
	std::vector<std::string> image_topics_info;
	std::vector<std::string> image_topics;
	std::vector<std::string> pointcloud_topics;
	std::vector<std::string> image_frame_ids;
	std::vector<std::string> pointcloud_frame_ids;
};

template <typename MessageT, typename MessageTConstPtr>
class Generic_sync_publisher {
	typedef kitti_exportConfig Config;
	typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
public:
	Generic_sync_publisher(ros::NodeHandle nh, ros::NodeHandle nh_private);
	virtual ~Generic_sync_publisher();

	virtual void init_param();
	virtual void print_param();
	virtual void init();
	virtual void callback(const MessageTConstPtr &message);
	virtual void reconfigure_callback(Config &config, uint32_t level);
	virtual void tf_export();
	virtual void camera_list_export();

	ros::Subscriber sub;
	ros::Publisher pub;

	boost::shared_ptr<tf::TransformListener> listener_transform;
	boost::shared_ptr<ReconfigureServer> reconfigure_server;

	std::vector<boost::shared_ptr<Camera_subscriber> > camera_nodes;
	std::vector<boost::shared_ptr<Pointcloud_subscriber> > pointcloud_nodes;

	bool calibration_exported;
	bool tf_exported;

	unsigned int sequence;

	ros::NodeHandle nh;
	ros::NodeHandle nh_private;

	Config_list config_processed;
	Config config;
};

template <typename MessageT, typename MessageTConstPtr>
Generic_sync_publisher<MessageT, MessageTConstPtr>::Generic_sync_publisher(ros::NodeHandle nh, ros::NodeHandle nh_private) {
	calibration_exported = false;
	tf_exported = false;
	sequence = 0;
	this->nh_private = nh_private;
	this->nh = nh;
}

template <typename MessageT, typename MessageTConstPtr>
Generic_sync_publisher<MessageT, MessageTConstPtr>::~Generic_sync_publisher() {
	// TODO Auto-generated destructor stub
}

template <typename MessageT, typename MessageTConstPtr>
void
Generic_sync_publisher<MessageT, MessageTConstPtr>::init(){
	// Set up dynamic reconfigure
	reconfigure_server.reset(new ReconfigureServer(nh_private));
	ReconfigureServer::CallbackType f = boost::bind(&Generic_sync_publisher<MessageT, MessageTConstPtr>::reconfigure_callback, this, _1, _2);
	reconfigure_server->setCallback(f);


	init_param();
	print_param();

	listener_transform.reset(new tf::TransformListener(nh, ros::Duration(config.tf_buffer_length)));

	// init camera nodes;
	camera_nodes.resize(config_processed.image_topics.size());
	for(int i = 0; i < config_processed.image_topics.size(); ++i){
		camera_nodes.at(i).reset( new Camera_subscriber() );
		camera_nodes.at(i)->init(nh, config_processed.image_topics.at(i), config.data_prefix, i, config.queue_size, config.publish_topic);
		camera_nodes.at(i)->create_image_info_sub(nh, config_processed.image_topics_info.at(i), config.queue_size);
		if( !config_processed.image_frame_ids.at(i).empty()){
			camera_nodes.at(i)->setFrame_id(config_processed.image_frame_ids.at(i));
		}
	}

	// init pointcloud nodes
	pointcloud_nodes.resize(config_processed.pointcloud_topics.size());
	for(int i = 0; i < config_processed.pointcloud_topics.size(); ++i){
		std::stringstream folder_name;
		folder_name << "velodyne_points";
		if(i != 0){ // Compatibility reasons
			folder_name << i;
		}
		pointcloud_nodes.at(i).reset(new Pointcloud_subscriber());
		pointcloud_nodes.at(i)->init(nh, config_processed.pointcloud_topics.at(i), config.data_prefix,
				folder_name.str(), config.queue_size);

		if( !config_processed.pointcloud_frame_ids.at(i).empty()){
			pointcloud_nodes.at(i)->setFrame_id(config_processed.pointcloud_frame_ids.at(i));
		}
	}

//
//	// todo: init imu nodes
//	//
//	//
//



	pub = nh.advertise<Sync_msg>(config.publish_topic, 1);
	sub = nh.subscribe<MessageT>(config.sync_topic, 1, boost::bind(&Generic_sync_publisher<MessageT,MessageTConstPtr>::callback, this, _1));

}

template <typename MessageT, typename MessageTConstPtr>
void
Generic_sync_publisher<MessageT, MessageTConstPtr>::init_param(){
	nh_private.getParam("subscribe_topics_image", config_processed.image_topics);
	nh_private.getParam("subscribe_topics_image_info", config_processed.image_topics_info);
	nh_private.getParam("camera_frame_ids", config_processed.image_frame_ids);
	nh_private.getParam("subscribe_topics_pcl", config_processed.pointcloud_topics);
	nh_private.getParam("pcl_frame_ids", config_processed.pointcloud_frame_ids);
	nh_private.getParam("publish", config.enabled);
	nh_private.getParam("sequence",config.sequence);
	nh_private.getParam("sync_topic",config.sync_topic);

	assert(config.data_prefix.length() > 1); // dont allow to write to root "/"

	assert(config_processed.image_topics.size() == config_processed.image_topics_info.size() );
	assert(config_processed.image_topics.size() > 0);
	assert(config_processed.image_frame_ids.size() <= config_processed.image_topics.size());

	assert(config_processed.pointcloud_topics.size() > 0);
	assert(config_processed.pointcloud_frame_ids.size() <= config_processed.pointcloud_topics.size());

	// Resize arrays to same size
	config_processed.image_topics.resize(config_processed.image_topics.size());
	config_processed.image_frame_ids.resize(config_processed.image_topics.size());
	config_processed.pointcloud_frame_ids.resize(config_processed.pointcloud_topics.size());
}

template <typename MessageT, typename MessageTConstPtr>
void
Generic_sync_publisher<MessageT, MessageTConstPtr>::print_param(){
	ROS_INFO("sync_topic:\t %s", config.sync_topic.c_str());
	ROS_INFO("Sequence:\t %u", config.sequence);
	ROS_INFO("publish_topic:\t %s", config.publish_topic.c_str());
	ROS_INFO("data_prefix:\t %s", config.data_prefix.c_str());
	ROS_INFO("tf_buffer_length:\t %d", config.tf_buffer_length);
	ROS_INFO("queue_size:\t %d", config.queue_size);
	ROS_INFO("Enabled:\t %d", config.enabled);
}

template <typename MessageT, typename MessageTConstPtr>
void
Generic_sync_publisher<MessageT, MessageTConstPtr>::tf_export(){

	assert(config_processed.image_frame_ids.size() == camera_nodes.size());

	bool all_valid = true;

	// Check if all camera nodes have received a calibration msg
	for(int i = 0; i < camera_nodes.size(); ++i){
		if(!camera_nodes.at(i)->calibration_valid){
			all_valid = false;
			return;
		}
	}

	if(!pointcloud_nodes.at(0)->valid ) return;

	ros::Time time = ros::Time::now();
	// Set frame ids, look up and set transforms cam0_to_camX
	for(int i = 0; i < camera_nodes.size(); ++i){

		// if its not camera 0 look up transform from 0 to cam i
		tf::StampedTransform tf;
		if(i != 0){
			try{
				std::string source_frame = camera_nodes.at(0)->calibration.frame_id.c_str();
				std::string target_frame = camera_nodes.at(i)->calibration.frame_id.c_str();
				std::string tf_error;
				printf("looking from tf %s to %s\n", source_frame.c_str(), target_frame.c_str());

				if(!listener_transform->waitForTransform( target_frame.c_str(), source_frame.c_str(), time, ros::Duration(3.0), ros::Duration(0.1), &tf_error)){
					printf("Kitti export: Tf error %s", tf_error.c_str());
					return;
				}
				listener_transform->lookupTransform(
					camera_nodes.at(i)->calibration.frame_id.c_str(), /* target frame */
					camera_nodes.at(0)->calibration.frame_id.c_str(), /* source frame  */
					time,	/* time */
					tf); /* transfrom */
			}catch (std::exception &e){
				ROS_WARN("Kitti export: transform look up failed: %s", e.what());
				return;
			}

			//printf("transform: %f %f %f rot: %f %f %f\n", tf.getOrigin()[0],tf.getOrigin()[1],tf.getOrigin()[2], tf.getRotation()[0], tf.getRotation()[1], tf.getRotation()[2], tf.getRotation()[3]);
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

		try{
			std::string source_frame = config_processed.pointcloud_frame_ids.at(0).c_str();
			std::string target_frame = camera_nodes.at(0)->calibration.frame_id.c_str();
			std::string tf_error;

			if(source_frame.empty())
			{
				source_frame = pointcloud_nodes.at(0)->frame_id.c_str();
			}

			printf("looking from tf %s to %s\n", source_frame.c_str(), target_frame.c_str());


			if(!listener_transform->waitForTransform( target_frame.c_str(), source_frame.c_str(), time, ros::Duration(3.0), ros::Duration(0.1), &tf_error)){
				printf("Kitti export: Tf error %s", tf_error.c_str());
				return;
			}

			listener_transform->lookupTransform(
								source_frame, /* target frame */
								target_frame, /* source frame  */
								time,	/* time */
								tf_velo_to_camera0);
		}
		catch(std::exception &e)
		{
				ROS_WARN("Kitti export: transform look up failed: %s", e.what());
				return;
		}
		kitti::filenames::save_tf_velo_to_camera0(config.data_prefix, tf_velo_to_camera0);
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
//			save.save_tf_velo_to_camera0(config.data_prefix, tf_velo_to_camera0);
//		}
	}
	tf_exported = true;

}

template <typename MessageT, typename MessageTConstPtr>
void
Generic_sync_publisher<MessageT, MessageTConstPtr>::camera_list_export(){
	// Export camera calibrations calib_cam_to_cam.txt
	Camera_list cam_list;

	for(int i = 0; i < camera_nodes.size(); ++i){
		cam_list.cameras.push_back(camera_nodes.at(i)->calibration);
	}

	if( filenames::save_camera_list(config.data_prefix, cam_list) ) {
		calibration_exported = true;
	}
	else{
		ROS_WARN("camera calibration export failed");
	}
}

template <typename MessageT, typename MessageTConstPtr>
void
Generic_sync_publisher<MessageT, MessageTConstPtr>::reconfigure_callback(Config &config, uint32_t level){
	this->config = config;
	print_param();
	return;
}

template <typename MessageT, typename MessageTConstPtr>
void
Generic_sync_publisher<MessageT, MessageTConstPtr>::callback(const MessageTConstPtr &message){
	if(!config.enabled) return; //if publishing not enabled do nothing
			kitti::Sync_msg sync_msg;

			sync_msg.header.seq = sequence; 	++sequence;
			sync_msg.header.stamp = message->header.stamp;

			pub.publish(sync_msg);

			if(!tf_exported){
				tf_export();
			}
	}

} /* namespace kitti */

#endif /* SRC_KITTI_EXPORT_COMMON_SYNC_H_ */
