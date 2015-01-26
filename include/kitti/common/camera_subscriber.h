/*
 * camera.h
 *
 *  Created on: 23.01.2015
 *      Author: fnolden
 */
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>


#include <common/generic_subscriber.h>
#include <common/serialization/filenames.h>

#include <kitti/Sync_msg.h>


#ifndef SRC_KITTI_CAMERA_SUBSCRIBER_H_
#define SRC_KITTI_CAMERA_SUBSCRIBER_H_

namespace kitti {

class Camera_subscriber : public Generic_subscriber<sensor_msgs::Image, sensor_msgs::ImageConstPtr> {
public:
	Camera_subscriber();
	virtual ~Camera_subscriber();

	virtual bool init(ros::NodeHandle &nh, std::string topic_name, std::string data_root, int camera_nr = 0, int queue_size = 30, std::string topic_sync = "/kitti/sync");
	void create_image_info_sub(ros::NodeHandle &nh, std::string topic, int queue_size);

	void callback(const sensor_msgs::ImageConstPtr &image, const Sync_msgConstPtr &sync);
	void callback_info(const sensor_msgs::CameraInfoConstPtr &info);

	ros::Subscriber sub_info;

	bool calibration_valid;
	kitti::Camera calibration;
};

} /* namespace kitti */

#endif /* SRC_KITTI_CAMERA_SUBSCRIBER_H_ */
