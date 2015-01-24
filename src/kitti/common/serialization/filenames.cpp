/*
 * kitti_store.cpp
 *
 *  Created on: 22.01.2015
 *      Author: fnolden
 */

#include <common/serialization/filenames.h>

namespace kitti {

bool
filenames::save_image_file(std::string data_root, cv::Mat image, int sequence, int camera_nr, uint sec, uint nsec){
	std::stringstream stream;

	char folder[8];
	sprintf(folder,"image_%02d/", camera_nr);

	stream << data_root << folder;

	save_timestamp(stream.str(), sequence, sec, nsec);

	stream << filenumber(sequence) << ".png";

	return cv::imwrite(stream.str(), image);
}

bool
filenames::save_camera_list(std::string data_root, Camera_list &camera_list){
	make_folder(data_root);

	std::string Filenames = data_root +"/calib_cam_to_cam.txt";

	return camera_list.save_file(Filenames);
}

bool
filenames::save_tf_velo_to_camera0(std::string data_root, Tf &kitti_tf){
	make_folder(data_root);

	std::string Filenames = data_root +"/velo_to_cam.txt";

	return kitti_tf.save_file(Filenames);
}

bool
filenames::save_tf_velo_to_camera0(std::string data_root, tf::Transform &tf){
	kitti::Tf kitti_tf;
	kitti_tf.set_transform(tf);

	return save_tf_velo_to_camera0(data_root, kitti_tf);
}

void
filenames::save_timestamp(std::string folder, unsigned int sequence, uint sec, uint nsec){
	std::ofstream outfile;

	folder.append("/").append(filenumber(sequence)).append(".txt");
	outfile.open(folder.c_str(), std::ios_base::app);
	outfile << sec << " " << nsec << "\n";
}

template <typename PointT>
bool
filenames::save_pointcloud(std::string data_root, pcl::PointCloud<PointT> &pointcloud, unsigned int sequence, uint sec, uint nsec, std::string folder_name){
	std::stringstream stream;

	make_folder(data_root);

	stream << data_root << folder_name <<"/";

	save_timestamp(stream.str(), sequence, sec, nsec);

	stream << "data/" << filenumber(sequence) << ".psd";

	pcl::io::savePCDFileBinary(stream.str(), pointcloud);

	return true;
}

bool
filenames::make_folder(std::string &path)
{
	if(path[path.length()-1] == '/') return true;
	else{
		path.append("/");
	}
	return true;
}

std::string
filenames::filenumber(unsigned int sequence){
	char image_name[10];
	sprintf(image_name,"/%010u.pcd", sequence);
	return std::string(image_name);
}

} /* namespace kitti */
