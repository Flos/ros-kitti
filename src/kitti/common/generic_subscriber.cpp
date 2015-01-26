/*
 * subscriber.cpp
 *
 *  Created on: 23.01.2015
 *      Author: fnolden
 */

#include <common/generic_subscriber.h>

namespace kitti {

template class Generic_subscriber<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2ConstPtr>;
template class Generic_subscriber<sensor_msgs::Image, sensor_msgs::ImageConstPtr>;

} /* namespace kitti */
