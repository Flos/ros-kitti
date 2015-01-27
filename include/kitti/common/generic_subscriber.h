/*
 * subscriber.h
 *
 *  Created on: 23.01.2015
 *      Author: fnolden
 */
#include <ros/ros.h>
#include <kitti/Sync_msg.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <common/serialization/filenames.h>

#ifndef SRC_KITTI_EXPORT_COMMON_GENERIC_SUBSCRIBER_H_
#define SRC_KITTI_EXPORT_COMMON_GENERIC_SUBSCRIBER_H_

namespace kitti {


template <typename Message_Type, typename Message_Type_Callback>
class Generic_subscriber {
	typedef message_filters::sync_policies::ApproximateTime<Message_Type, Sync_msg> Filter;
public:
	Generic_subscriber(){ valid = false; };
	virtual ~Generic_subscriber(){};
	virtual void callback(const Message_Type_Callback &message, const Sync_msgConstPtr &header) = 0;
	std::string data_root;
	std::string frame_id;
	bool valid;

	virtual void setFrame_id(std::string frame_id){
		if(!valid){
			this->frame_id = frame_id;
			valid = true;
		}
	}

protected:
	virtual void init(ros::NodeHandle &nh, std::string topic_name, std::string data_root, int queue_size = 30, std::string topic_sync = "/kitti/sync"){

		this->data_root = data_root;
		if(topic_name.empty()){ return;};

			sub.reset( new message_filters::Subscriber<Message_Type>( nh, topic_name,queue_size));
			sub_header.reset( new message_filters::Subscriber<Sync_msg>( nh, topic_sync, queue_size));

			// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
			sync.reset(	new message_filters::Synchronizer<Filter>( Filter(queue_size), *sub, *sub_header));
			sync->registerCallback(	boost::bind(&Generic_subscriber::callback, this, _1, _2));
	}


	boost::shared_ptr<message_filters::Subscriber<Message_Type> > sub;
	boost::shared_ptr<message_filters::Subscriber<Sync_msg> > sub_header;
	boost::shared_ptr<message_filters::Synchronizer<Filter> > sync;
};

} /* namespace kitti */

#endif /* SRC_KITTI_EXPORT_COMMON_GENERIC_SUBSCRIBER_H_ */
