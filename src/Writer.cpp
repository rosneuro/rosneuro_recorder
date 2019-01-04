#ifndef ROSNEURO_RECORDER_WRITER_CPP
#define ROSNEURO_RECORDER_WRITER_CPP

#include "rosneuro_recorder/Writer.hpp"

namespace rosneuro {

Writer::Writer(NeuroFrame* frame) {
	this->name_	 = "undefined";
	this->frame_ = frame;
}

Writer::~Writer(void) {
}

std::string Writer::GetName(void) {
	return this->name_;
}

void Writer::Who(void) {
	printf("[%s] - %s writer\n", this->GetName().c_str(), this->GetName().c_str());
}

}


#endif
