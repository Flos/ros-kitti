/*
 * subscriber.cpp
 *
 *  Created on: 23.01.2015
 *      Author: fnolden
 */

#include <common/generic_subscriber.h>

namespace kitti {


template <typename Message_Type>
Generic_subscriber<Message_Type>::Generic_subscriber() {
}

template <typename Message_Type>
Generic_subscriber<Message_Type>::~Generic_subscriber() {
	// TODO Auto-generated destructor stub
}

template <typename Message_Type>
void
Generic_subscriber<Message_Type>::init(ros::NodeHandle &nh, std::string topic_name, std::string data_root, int queue_size, std::string topic_sync){
	this->data_root = data_root;
	if(topic_name.empty()){ return;};

		sub.reset( new message_filters::Subscriber<Message_Type>( nh, topic_name,queue_size));
		sub_header.reset( new message_filters::Subscriber<kitti::Sync_msg>( nh, topic_sync, queue_size));

		// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
		sync.reset(	new message_filters::Synchronizer<Filter>( Filter(queue_size, *sub, *sub_header)));
		sync->registerCallback(	boost::bind(&Generic_subscriber::callback, this, _1, _2));
}

} /* namespace kitti */
