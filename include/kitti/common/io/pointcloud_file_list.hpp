/*
 * Filelist.h
 *
 *  Created on: 19.01.2015
 *      Author: fnolden
 */
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <time.h>
#include <stdio.h>
#include <assert.h>
#include <vector>

#include <pcl/io/file_io.h>

#include <kitti/common/serialization/serializable.h>
#include <kitti/common/serialization/filenames.h>
#include <kitti/common/io/pointcloud_io.hpp>

#ifndef IO_POINTCLOUD_FILELIST_H_
#define IO_POINTCLOUD_FILELIST_H_

namespace kitti {

template <typename PointT>
class Pointcloud_file_list: public String_list {
public:
	Pointcloud_file_list(){};
	virtual ~Pointcloud_file_list(){};

	bool load_pointcloud(pcl::PointCloud<PointT> &cloud, int index){
		return io::load_pointcloud(path + String_list::list.at(index), cloud);
	}

	bool save_pointcloud(const pcl::PointCloud<PointT> &cloud, int index){
			return io::save_pointcloud(path + String_list::list.at(index), cloud);
	}

	bool save_pointcloud(const pcl::PointCloud<PointT> &cloud, int index, unsigned int sec, unsigned int nsec){
		return filenames::save_pointcloud<PointT>(path, cloud, index, sec, nsec);
	}
};

} /* namespace kitti */

#endif /* SRC_GUI_FILELIST_H_ */
