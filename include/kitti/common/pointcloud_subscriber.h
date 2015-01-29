/*
 * pointcloud_subscriber.h
 *
 *  Created on: 23.01.2015
 *      Author: fnolden
 */

#ifndef INCLUDE_KITTI_POINTCLOUD_SUBSCRIBER_H_
#define INCLUDE_KITTI_POINTCLOUD_SUBSCRIBER_H_

#include <kitti/common/generic_subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <kitti/common/serialization/filenames.h>

namespace kitti {

class Pointcloud_subscriber : public Generic_subscriber<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2ConstPtr>{
	typedef sensor_msgs::PointCloud2 Message_Type;
	typedef sensor_msgs::PointCloud2ConstPtr Message_Type_Callback;
	typedef message_filters::sync_policies::ApproximateTime<Message_Type, Message_Type> Filter;
public:
	Pointcloud_subscriber();
	virtual ~Pointcloud_subscriber();
	bool init(ros::NodeHandle &nh, std::string topic_name, std::string data_root,
				std::string folder_name, int queue_size, std::string topic_sync = "/kitti/sync");
	void callback(const Message_Type_Callback &message, const Sync_msgConstPtr &header);

	boost::shared_ptr<tf::TransformListener> listener_transform;
	std::string folder_name;
};

} /* namespace kitti */

#endif /* INCLUDE_KITTI_POINTCLOUD_SUBSCRIBER_H_ */
