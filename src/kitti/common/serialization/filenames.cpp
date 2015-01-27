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

	char folder[9];
	sprintf(folder,"image_%02d/", camera_nr);

	stream << data_root << folder;

	create_folder(stream.str());

	save_timestamp(stream.str(), sequence, sec, nsec);

	stream << "data/";

	create_folder(stream.str());

	stream << filenumber(sequence) << ".png";

	return cv::imwrite(stream.str(), image);
}

bool
filenames::save_camera_list(std::string data_root, Camera_list &camera_list){
	std::string Filenames = data_root +"calib_cam_to_cam.txt";

	return camera_list.save_file(Filenames);
}

bool
filenames::save_tf_velo_to_camera0(std::string data_root, Tf &kitti_tf){
	return save_tf(data_root, "calib_velo_to_cam.txt", kitti_tf);
}

bool
filenames::save_tf_velo_to_camera0(std::string data_root, tf::Transform &tf){
	kitti::Tf kitti_tf;
	kitti_tf.set_transform(tf);

	return save_tf(data_root, "calib_velo_to_cam.txt", kitti_tf);
}

bool
filenames::save_tf(std::string data_root, std::string filename, Tf &kitti_tf){
	std::string Filenames = data_root + filename;

	return kitti_tf.save_file(Filenames);
}

bool
filenames::save_tf(std::string data_root, std::string filename, tf::Transform &tf){
	kitti::Tf kitti_tf;
	kitti_tf.set_transform(tf);

	return save_tf(data_root, filename, kitti_tf);
}

void
filenames::save_timestamp(std::string folder, unsigned int sequence, uint sec, uint nsec){
	std::ofstream outfile;

	folder.append("/").append(filenumber(sequence)).append(".txt");
	outfile.open(folder.c_str(), std::ios_base::app);
	outfile << sec << " " << nsec << "\n";
}

bool
filenames::save_pointcloud(std::string data_root, pcl::PointCloud<pcl::PointXYZI> &pointcloud, unsigned int sequence, uint sec, uint nsec, std::string folder_name){
	std::stringstream stream;

	stream << data_root << folder_name <<"/";

	save_timestamp(stream.str(), sequence, sec, nsec);

	stream << "data/";

	create_folder(stream.str());

	stream << filenumber(sequence) << ".pcd";

	pcl::io::savePCDFileBinary(stream.str(), pointcloud);

	return true;
}

bool
filenames::make_folder_slash(std::string &_filePath)
{
	if(_filePath[_filePath.length()-1] != '/')
	{
		_filePath.append("/");
	}

	create_folder(_filePath);

	return true;
}

bool
filenames::create_folder(std::string filePath)
{

	boost::filesystem::path dir(filePath);
	if(boost::filesystem::create_directories(dir))
	{
		std::cerr<< "Directory Created: "<< filePath << std::endl;
	}

	return true;
}

std::string
filenames::filenumber(unsigned int sequence){
	char image_name[10];
	sprintf(image_name,"/%010u", sequence);
	return std::string(image_name);
}

} /* namespace kitti */
