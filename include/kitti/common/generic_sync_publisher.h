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
#include <tf/transform_listener.h>

#include <common/serialization/filenames.h>
#include <common/serialization/camera.h>
#include <common/camera_subscriber.h>
#include <common/pointcloud_subscriber.h>


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

template <typename MessageT>
class Generic_sync_publisher {
	typedef kitti_exportConfig Config;
	typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
public:
	Generic_sync_publisher(ros::NodeHandle nh, ros::NodeHandle nh_private);
	virtual ~Generic_sync_publisher();

	virtual void init_param();
	virtual void init();
	virtual void callback(const MessageT &message);
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

	Config_list config_processd;
	Config config;
};

} /* namespace kitti */

#endif /* SRC_KITTI_EXPORT_COMMON_SYNC_H_ */
