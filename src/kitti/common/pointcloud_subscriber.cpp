/*
 * pointcloud_subscriber.cpp
 *
 *  Created on: 23.01.2015
 *      Author: fnolden
 */

#include <common/pointcloud_subscriber.h>

namespace kitti {

Pointcloud_subscriber::Pointcloud_subscriber() {
	valid = false;
}

Pointcloud_subscriber::~Pointcloud_subscriber() {
	// TODO Auto-generated destructor stub
}

void
Pointcloud_subscriber::callback(const Message_Type_Callback &message, const Sync_msgConstPtr &header) {
	frame_id = message->header.frame_id;

	//printf("poitncloud callback frame_id: %s, %s, %d\n", frame_id.c_str(), message->header.frame_id.c_str(), valid);


	pcl::PointCloud<pcl::PointXYZI> cloud;
	pcl::fromROSMsg(*message, cloud);

	filenames::save_pointcloud(data_root, cloud, header->header.seq, message->header.stamp.sec, message->header.stamp.nsec, folder_name);
	valid = true;
}

bool
Pointcloud_subscriber::init(ros::NodeHandle &nh, std::string topic_name, std::string data_root, std::string folder_name, int queue_size, std::string topic_sync ) {
	Generic_subscriber::init(nh, topic_name, data_root, queue_size, topic_sync);
	this->folder_name = folder_name;

	return true;
}

} /* namespace kitti */
