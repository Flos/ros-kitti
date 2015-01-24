/*
 * kitti_data_store.h
 *
 *  Created on: 22.01.2015
 *      Author: fnolden
 */

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <common/serialization/camera_list.h>
#include <common/serialization/tf.h>
#include <tf/tf.h>
#include <string>
#include <stdio.h>
#include <pcl/io/pcd_io.h>

#ifndef INCLUDE_KITTI_COMMON_FILENAMES_H_
#define INCLUDE_KITTI_COMMON_FILENAMES_H_

namespace kitti {

namespace filenames {

	bool save_image_file(std::string data_root, cv::Mat image, int sequence, int camera_nr, uint sec, uint nsec);
	//bool save_image_file(std::string data_root, int sequence, int camera_nr, uint sec, uint nsec);
	bool save_camera_list(std::string data_root, Camera_list &camera_list);
	bool save_tf_velo_to_camera0(std::string data_root, Tf &kitti_tf);
	bool save_tf_velo_to_camera0(std::string data_root, tf::Transform &tf);
	template <typename PointT>
	bool save_pointcloud(std::string data_root, pcl::PointCloud<PointT> &pointcloud, unsigned int sequence, uint sec, uint nsec, std::string folder_name="velodyne_points");
	void save_timestamp(std::string folder, unsigned int sequence, uint sec, uint nsec);
	std::string filenumber(unsigned int sequence);
	bool make_folder(std::string &path);
};


} /* namespace kitti */

#endif /* INCLUDE_KITTI_COMMON_FILENAMES_H_ */
