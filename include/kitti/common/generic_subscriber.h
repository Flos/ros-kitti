/*
 * subscriber.h
 *
 *  Created on: 23.01.2015
 *      Author: fnolden
 */
#include <ros/ros.h>
#include <kitti/Sync_msg.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <common/serialization/filenames.h>

#ifndef SRC_KITTI_EXPORT_COMMON_SUBSCRIBER_H_
#define SRC_KITTI_EXPORT_COMMON_SUBSCRIBER_H_

namespace kitti {


template <typename Message_Type>
class Generic_subscriber {
	typedef message_filters::sync_policies::ApproximateTime<Message_Type, std_msgs::Header> Filter;
public:
	Generic_subscriber();
	virtual ~Generic_subscriber();
	virtual void callback(const Message_Type &message, const kitti::Sync_msgConstPtr &header) = 0;

protected:
	virtual void init(ros::NodeHandle &nh, std::string topic_name, std::string data_root, int queue_size = 30, std::string topic_sync = "/kitti/sync");

	std::string data_root;
	std::string frame_id;
	boost::shared_ptr<message_filters::Subscriber<Message_Type> > sub;
	boost::shared_ptr<message_filters::Subscriber<kitti::Sync_msg> > sub_header;
	boost::shared_ptr<message_filters::Synchronizer<Filter> > sync;
};

} /* namespace kitti */

#endif /* SRC_KITTI_EXPORT_COMMON_SUBSCRIBER_H_ */
