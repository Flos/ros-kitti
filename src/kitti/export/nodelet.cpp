#include <stdio.h>

#include <common/generic_sync_publisher.h>
#include <sensor_msgs/PointCloud2.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace kitti{
	class Export_nodelet: public nodelet::Nodelet
	{

	public:
		Export_nodelet(){}
		~Export_nodelet(){}
	   virtual void onInit(){
		   inst_.reset(new Generic_sync_publisher<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2ConstPtr>(getNodeHandle(), getPrivateNodeHandle()));
		   inst_->init();
	   }
	   boost::shared_ptr<Generic_sync_publisher<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2ConstPtr> > inst_;

	};
}

PLUGINLIB_DECLARE_CLASS(kitti, Export_nodelet, kitti::Export_nodelet, nodelet::Nodelet)
