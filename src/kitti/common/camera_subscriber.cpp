/*
 * camera.cpp
 *
 *  Created on: 23.01.2015
 *      Author: fnolden
 */


#include <kitti/common/camera_subscriber.h>

namespace kitti {

Camera_subscriber::Camera_subscriber() {
	calibration_valid = false;
}
Camera_subscriber::~Camera_subscriber() {};

bool
Camera_subscriber::init(ros::NodeHandle &nh, std::string topic_name, std::string data_root, int camera_nr, int queue_size, std::string topic_sync){
	Generic_subscriber::init(nh, topic_name, data_root, queue_size, topic_sync);
	calibration.set_camera_nr(camera_nr);

	return true;
}

void
Camera_subscriber::setFrame_id(std::string frame_id){
	calibration.frame_id = frame_id;
	Generic_subscriber::setFrame_id(frame_id);
}

void
Camera_subscriber::create_image_info_sub( ros::NodeHandle &nh, std::string topic, int queue_size){
	sub_info = nh.subscribe<sensor_msgs::CameraInfo>(topic, queue_size, boost::bind(&Camera_subscriber::callback_info, this, _1) );
}

void
Camera_subscriber::callback(const sensor_msgs::ImageConstPtr &image, const Sync_msgConstPtr &sync){
	cv_bridge::CvImagePtr cv_bridge_image = cv_bridge::toCvCopy(image);
	filenames::save_image_file(data_root, cv_bridge_image->image, sync->header.seq, calibration.camera_nr, image->header.stamp.sec, image->header.stamp.nsec );
}

void
Camera_subscriber::callback_info(const sensor_msgs::CameraInfoConstPtr &info){
	calibration.set_camera_info(*info);

	if(!valid){
		setFrame_id(info->header.frame_id);
	}
	else{
		setFrame_id(frame_id); // overwrite camera_info "frame_id" with the set
	}
	calibration_valid = true;

	//unsubscribe after retrieving the calibration
	sub_info.shutdown();
}

} /* namespace kitti */

