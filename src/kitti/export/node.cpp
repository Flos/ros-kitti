#include <stdio.h>

#include <kitti/common/generic_sync_publisher.h>
#include <kitti/common/camera_subscriber.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char** argv){
	printf("Starting kitti_export \n");

	ros::init(argc, argv, "kitti_export");
	ros::NodeHandle node;
	ros::NodeHandle priv_nh("~");


	kitti::Generic_sync_publisher<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2ConstPtr> kitti_export (node, priv_nh);
	kitti_export.init();

	//sub.init(node, "/image1","/tmp/",1, 5, "/sync");

	ros::Rate rate(60);
	while(node.ok()){
		ros::spinOnce();
		rate.sleep();
	}

	printf("kitti_export stopped \n");
	return 0;
}
