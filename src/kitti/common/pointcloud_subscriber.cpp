/*
 * pointcloud_subscriber.cpp
 *
 *  Created on: 23.01.2015
 *      Author: fnolden
 */

#include <kitti/common/pointcloud_subscriber.h>

namespace kitti {

Pointcloud_subscriber::Pointcloud_subscriber() {
}

Pointcloud_subscriber::~Pointcloud_subscriber() {
	// TODO Auto-generated destructor stub
}

void
Pointcloud_subscriber::callback(const Message_Type_Callback &message, const Sync_msgConstPtr &header) {
	if(!valid){
		Generic_subscriber::setFrame_id( message->header.frame_id);
	}
	//printf("poitncloud callback frame_id: %s, %s, %d\n", frame_id.c_str(), message->header.frame_id.c_str(), valid);

	pcl::PointCloud<pcl::PointXYZI> cloud;
	pcl::fromROSMsg(*message, cloud);


	if (!boost::equals(frame_id, message->header.frame_id)){
		std::string tf_error;
		if(!listener_transform->waitForTransform( frame_id.c_str(),
													message->header.frame_id.c_str(),
													message->header.stamp,
													ros::Duration(5.0),
													ros::Duration(0.1),
													&tf_error)){
			ROS_WARN("Kitti export: pointcloud Tf error %s", tf_error.c_str());
			return;
		}
		if (!pcl_ros::transformPointCloud(frame_id, cloud, cloud, *listener_transform)) {
			ROS_WARN("Cannot transform point cloud to the fixed frame %s.", frame_id.c_str());
			return;
		}
	}
	filenames::save_pointcloud(data_root, cloud, header->header.seq, message->header.stamp.sec, message->header.stamp.nsec, folder_name);
}

bool
Pointcloud_subscriber::init(ros::NodeHandle &nh, std::string topic_name, std::string data_root, std::string folder_name, int queue_size, std::string topic_sync ) {
	listener_transform.reset(new tf::TransformListener(nh, ros::Duration(queue_size)));
	Generic_subscriber::init(nh, topic_name, data_root, queue_size, topic_sync);
	this->folder_name = folder_name;

	return true;
}

} /* namespace kitti */
