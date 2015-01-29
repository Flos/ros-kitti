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

#include <opencv2/opencv.hpp>

#include <kitti/common/serialization/serializable.h>
#include <kitti/common/serialization/filenames.h>
#include <kitti/common/io/image_io.hpp>

#ifndef IO_IMAGE_FILELIST_H_
#define IO_IMAGE_FILELIST_H_

namespace kitti {

class Image_file_list: public String_list {
public:
	Image_file_list(){};
	virtual ~Image_file_list(){};

	bool load_image(cv::Mat &image, int index){
		  return io::load_image(path + String_list::list.at(index), image);
	}

	bool save_image(const cv::Mat &image, int index){
		return io::save_image(path + String_list::list.at(index), image);
	}

	bool save_image(const cv::Mat &image, int index, int camera, unsigned int sec, unsigned int nsec){
			return filenames::save_image_file(path, image, index, camera, sec, nsec);
	}

};

} /* namespace kitti */

#endif /* SRC_GUI_FILELIST_H_ */
