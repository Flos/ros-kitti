/*
 * Filelist.cpp
 *
 *  Created on: 19.01.2015
 *      Author: fnolden
 */

#include <kitti/common/serialization/file_list.h>

namespace kitti {

String_list::String_list() {
	// TODO Auto-generated constructor stub

}

String_list::~String_list() {
	// TODO Auto-generated destructor stub
}

void
String_list::get_fullname(std::string &filepath, int index){
	if(index > list.size()){
		std::cout << "Index: " << index << " out of bounds, Max: " << list.size() << "\n";
		return;
	}
	filepath = path + list.at(index);
}

std::string
String_list::get_fullname(int index){
	std::string fullname;
	get_fullname(fullname,  index);

	return fullname;
}

bool
String_list::load(std::istream &stream){
	std::string line;
	while ( std::getline (stream, line) )
	{
		list.push_back(line);
	}
	return true;
}

std::string
String_list::to_string(){
	std::stringstream ss;
	for(int i = 0; i < list.size(); ++i){
		ss << list.at(i) << new_line;
	}

	if(list.empty()){
		std::cout << "File is empty\n";
	}
	return ss.str();
}

unsigned int
String_list::size(){
	return list.size();
}

std::string
String_list::at(int idx){
	return list.at(idx);
}

} /* namespace image_cloud */
