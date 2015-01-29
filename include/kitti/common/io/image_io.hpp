#include <opencv2/highgui/highgui.hpp>

#ifndef KITTI_IO_IMAGE_H_
#define KITTI_IO_IMAGE_H_

namespace kitti{

namespace io{

	inline bool save_image(std::string full_filepath, const cv::Mat &image){
		try{
			return cv::imwrite(full_filepath, image);
		}catch(std::exception &e){
			printf("image save failed: %s\n", e.what());
			return false;
		}
	}

	inline bool load_image(std::string full_filepath, cv::Mat &image){
		try{
			image = cv::imread(full_filepath);
		}catch(std::exception &e){
			printf("image load failed: %s\n", e.what());
			return false;
		}
		return true;
	}
}

}

#endif
