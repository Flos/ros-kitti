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

#include <common/serialization/filenames.h>
#include <common/serialization/camera.h>
#include <common/camera_subscriber.h>
#include <common/pointcloud_subscriber.h>
#include <common/generic_subscriber.h>

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
	init_param();
	print_param();


	// init camera nodes;
	camera_nodes.resize(config_processed.image_topics.size());
	for(int i = 0; i < config_processed.image_topics.size(); ++i){
		camera_nodes.at(i).reset( new Camera_subscriber() );
		camera_nodes.at(i)->init(nh, config_processed.image_topics.at(i), config.data_destination, i, config.queue_size);
		camera_nodes.at(i)->create_image_info_sub(nh, config_processed.image_topics_info.at(i), config.queue_size);
	}

	// init pointcloud nodes
	pointcloud_nodes.resize(config_processed.pointcloud_topics.size());
	for(int i = 0; i < config_processed.pointcloud_topics.size(); ++i){
		std::stringstream folder_name;
		folder_name << "velodyne_points";
		if(i != 0){
			folder_name << i;
		}
		pointcloud_nodes.at(i).reset(new Pointcloud_subscriber());
		pointcloud_nodes.at(i)->init(nh, config_processed.pointcloud_topics.at(i), config.data_destination, folder_name.str(), config.queue_size);
	}
//
//	// todo: init imu nodes
//	//
//	//
//
	printf("init pub %s\n", config.sync_topic.c_str());


	listener_transform.reset(new tf::TransformListener(nh, ros::Duration(config.tf_buffer_length), true));
	printf("init pub %s\n", config.sync_topic.c_str());
	pub = nh.advertise<Sync_msg>(config.publish_topic, 1);
	sub = nh.subscribe<MessageT>(config.sync_topic,1, boost::bind(&Generic_sync_publisher<MessageT,MessageTConstPtr>::callback, this, _1));
	printf("init done\n");

	// Set up dynamic reconfigure
	reconfigure_server.reset(new ReconfigureServer(nh));
	ReconfigureServer::CallbackType f = boost::bind(&Generic_sync_publisher<MessageT, MessageTConstPtr>::reconfigure_callback, this, _1, _2);
	reconfigure_server->setCallback(f);
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

	// Set frame ids, look up and set transforms cam0_to_camX
	for(int i = 0; i < camera_nodes.size(); ++i){
		// if frame_id is provided use it instead
		if(!config_processed.image_frame_ids.at(i).empty()){
			camera_nodes.at(i)->calibration.frame_id = config_processed.image_frame_ids.at(i);
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

template <typename MessageT, typename MessageTConstPtr>
void
Generic_sync_publisher<MessageT, MessageTConstPtr>::camera_list_export(){
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
	printf("callback \n");
	if(!config.enabled) return; //if publishing not enabled do nothing
			kitti::Sync_msg sync_msg;

			sync_msg.header.seq = sequence; 	++sequence;
			sync_msg.header.stamp = message->header.stamp;

			pub.publish(sync_msg);

			if(!tf_exported){
				tf_export();
			}
			printf("callback processed\n");
	}

} /* namespace kitti */

#endif /* SRC_KITTI_EXPORT_COMMON_SYNC_H_ */
