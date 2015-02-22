#include <pcl/io/file_io.h>

#ifndef KITTI_IO_POINTCLOUD_H_
#define KITTI_IO_POINTCLOUD_H_

namespace kitti {

namespace io{

	template <typename PointT>
	inline bool save_pointcloud(std::string full_filepath, const pcl::PointCloud<PointT> &pointcloud){
		try{
			pcl::io::savePCDFileBinary(full_filepath, pointcloud);
		}catch(std::exception &e){
			return false;
		}
		return true;
	}

	template <typename PointT>
	inline bool load_pointcloud(std::string full_filepath, pcl::PointCloud<PointT> &pointcloud, bool verbose=false){
		try{
			// Check extension...
			int type = 0;
			if(	 full_filepath[full_filepath.length()-4] == '.'
					&& (full_filepath[full_filepath.length()-3] == 'b'
					|| full_filepath[full_filepath.length()-3] == 'B')
				)
			{
				type = 1;
			}

			switch (type) {
				default:
				case 0:
				{
					if (pcl::io::loadPCDFile<PointT> (full_filepath.c_str(), pointcloud) == -1) //* load the file
					{
						PCL_ERROR ("Couldn't read file %s \n", full_filepath.c_str() );
						return false;
					}
					break;
				}
				case 1:	// Todo: This part is not generic! always expects pcl::pointXYZI
				{
					std::fstream input(full_filepath.c_str(), std::ios::in | std::ios::binary);
					if(!input.good()){
						std::cout << "Could not read file: " << full_filepath << "\n";
					}
					else
					{
						input.seekg(0, std::ios::beg);
						int i;
						for (i=0; input.good() && !input.eof(); i++) {
							pcl::PointXYZI point;
							input.read((char *) &point.x, 3*sizeof(float));
							input.read((char *) &point.intensity, sizeof(float));
							pointcloud.push_back(point);
						}
						input.close();
					}
					break;
				}

			} /* switch end */
			if(verbose){
				std::cout << "Loaded [" << pointcloud.size() << "] points from file ["<< full_filepath << "]\n";
			}
		}catch(std::exception &e){
			return false;
		}

		return true;
	}

}

}
#endif
