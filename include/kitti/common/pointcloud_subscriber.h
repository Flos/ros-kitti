/*
 * pointcloud_subscriber.h
 *
 *  Created on: 23.01.2015
 *      Author: fnolden
 */

#ifndef INCLUDE_KITTI_POINTCLOUD_SUBSCRIBER_H_
#define INCLUDE_KITTI_POINTCLOUD_SUBSCRIBER_H_

#include <common/generic_subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

namespace kitti {

class Pointcloud_subscriber: public Generic_subscriber<sensor_msgs::PointCloud2ConstPtr> {
public:
	Pointcloud_subscriber();
	virtual ~Pointcloud_subscriber();
	bool init(ros::NodeHandle &nh, std::string topic_name, std::string data_root, std::string folder_name, int queue_size, std::string sync_topic = "/kitti/sync");
	void callback(const sensor_msgs::PointCloud2ConstPtr &message, const Sync_msgConstPtr &header);
	std::string folder_name;
};

} /* namespace kitti */

#endif /* INCLUDE_KITTI_POINTCLOUD_SUBSCRIBER_H_ */
