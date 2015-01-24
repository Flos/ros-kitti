/*
 * pointcloud_subscriber.cpp
 *
 *  Created on: 23.01.2015
 *      Author: fnolden
 */

#include <common/pointcloud_subscriber.h>

namespace kitti {

Pointcloud_subscriber::Pointcloud_subscriber() {

}

Pointcloud_subscriber::~Pointcloud_subscriber() {
	// TODO Auto-generated destructor stub
}

void
Pointcloud_subscriber::callback(const sensor_msgs::PointCloud2ConstPtr &message, const Sync_msgConstPtr &header) {
	frame_id = message->header.frame_id;

	pcl::PointCloud<pcl::PointXYZI> cloud;
	pcl::fromROSMsg(*message, cloud);

	filenames::save_pointcloud(data_root, cloud, header->header.seq, message->header.stamp.sec, message->header.stamp.nsec, folder_name);
}

bool
Pointcloud_subscriber::init(ros::NodeHandle &nh, std::string topic_name, std::string data_root, std::string folder_name, int queue_size, std::string sync_topic ) {
	Generic_subscriber::init(nh, topic_name, data_root, queue_size, sync_topic);

	this->folder_name = folder_name;
	return true;
}

} /* namespace kitti */
