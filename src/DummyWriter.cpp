#ifndef ROSNEURO_RECORDER_DUMMYWRITER_CPP
#define ROSNEURO_RECORDER_DUMMYWRITER_CPP

#include "rosneuro_recorder/DummyWriter.hpp"

namespace rosneuro {

DummyWriter::DummyWriter(NeuroFrame* frame) : Writer(frame) {
	this->name_ = "dummywriter";
}

DummyWriter::~DummyWriter(void) {
}

bool DummyWriter::Setup(void) {
	return true;
}

bool DummyWriter::Open(const std::string& filename) {
	return true;
}

bool DummyWriter::Close(void) {
	return true;
}

int DummyWriter::Write(int nswrite) {
	return 0;
}

bool DummyWriter::AddEvent(int event, double onset, double duration) {
	return true;
}

}


#endif
