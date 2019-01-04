#ifndef ROSNEURO_RECORDER_CPP
#define ROSNEURO_RECORDER_CPP

#include "rosneuro_recorder/Recorder.hpp"

namespace rosneuro {

Recorder::Recorder(void) : p_nh_("~") {
	this->topic_		 = "neurodata";
	this->is_configured_ = false;
}

Recorder::~Recorder(void) {

	this->writer_->Close();
}

bool Recorder::configure(void) {

	unsigned int wrttype = WriterType::XDFWRT;

	this->writer_ = factory.createWriter(&(this->frame_), wrttype);
	
	if(ros::param::get("~filename", this->filename_) == false) {
		this->filename_ = "/home/ltonin/" + this->get_datetime() + ".test.gdf";
	}

	printf("Filename %s\n", this->filename_.c_str()); 

	this->sub_ = this->nh_.subscribe(this->topic_, 1000, &Recorder::on_received_data, this);
	this->srv_info_ = this->nh_.serviceClient<rosneuro_msgs::GetAcquisitionInfo>("/acquisition/get_info");

	return true;
}

std::string Recorder::get_datetime(void) {
	time_t rawtime;
	struct tm * timeinfo;
	char buffer [80];

	time (&rawtime);
	timeinfo = localtime (&rawtime);

	strftime (buffer,80,"%Y%m%d.%H%M%S",timeinfo);

	return std::string(buffer);
}

void Recorder::on_received_data(const rosneuro_msgs::NeuroFrame& msg) {

	size_t gsize;
	int    wsize;

	gsize = msg.eeg.info.nsamples;

	if(this->IsConfigured() == false) {
		return;
	}

	// Convert message frame to message data
	NeuroDataTools::ToNeuroFrame(msg, this->frame_);

	// Write data
	wsize = this->writer_->Write(gsize);
	
	if(wsize == -1) {
		ROS_ERROR("Failed writing on file");
	} else if (wsize != gsize) {
		ROS_WARN("Not all data has been written");
	}

}

bool Recorder::IsConfigured(void) {
	return this->is_configured_;
}

bool Recorder::WaitForInfo(void) {
	
	ros::Rate r(60);
	rosneuro_msgs::GetAcquisitionInfo info;
	
	while( (this->srv_info_.call(info) == false) && ros::ok()) {
		ROS_WARN_THROTTLE(5.0f, "Cannot retrieve data info: waiting...");
		ros::spinOnce();
		r.sleep();
	}

	// Configure frame from service message
	if(NeuroDataTools::ConfigureNeuroFrame(info.response.frame, this->frame_) == false)
		ROS_ERROR("Cannot configure frame");
	
	// Reserve frame data
	this->frame_.eeg.reserve(this->frame_.eeg.nsamples(), this->frame_.eeg.nchannels());
	this->frame_.exg.reserve(this->frame_.exg.nsamples(), this->frame_.exg.nchannels());
	this->frame_.tri.reserve(this->frame_.tri.nsamples(), this->frame_.tri.nchannels());

	// Debug - Dump device configuration
	this->frame_.eeg.dump();
	this->frame_.exg.dump();
	this->frame_.tri.dump();

	this->is_configured_ = true;

}

bool Recorder::Run(void) {

	ros::Rate r(60);

	// Configure Recorder
	this->configure();
	ROS_INFO("Recorder succesfully configured");
	
	// Request device/data info (loop)
	this->WaitForInfo();
	ROS_INFO("Data information received from acquisition");
	
	// Open writer
	if(this->writer_->Open(this->filename_) == false) {
		ROS_ERROR("Cannot open the file: %s", this->filename_.c_str());
		return false;
	}
	ROS_INFO("File open: %s", this->filename_.c_str());

	// Setup Writer
	if(this->writer_->Setup() == false) {
		ROS_ERROR("Cannot setup the writer");
		return false;
	}
	ROS_INFO("Writer correctly setup");
	
	while(ros::ok()) {
		ros::spinOnce();
		r.sleep();
	}

	return true;


}

}


#endif
