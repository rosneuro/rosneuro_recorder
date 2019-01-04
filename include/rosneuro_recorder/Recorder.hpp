#ifndef ROSNEURO_RECORDER_HPP
#define ROSNEURO_RECORDER_HPP

#include <ros/ros.h>
#include "rosneuro_recorder/FactoryWriter.hpp"
#include "rosneuro_data/NeuroData.hpp"
#include "rosneuro_data/NeuroDataTools.hpp"
#include "rosneuro_msgs/NeuroFrame.h"
#include "rosneuro_msgs/NeuroDataInfo.h"
#include "rosneuro_msgs/GetAcquisitionInfo.h"

namespace rosneuro {

class Recorder {
	public:
		Recorder(void);
		virtual ~Recorder(void);

		bool configure(void);

		bool WaitForInfo(void);
		bool Run(void);

		bool IsConfigured(void);

		bool Start(void);
		bool Stop(void);

	private:
		void on_received_data(const rosneuro_msgs::NeuroFrame& msg);
		std::string get_datetime(void);

	private:
		ros::NodeHandle nh_;
		ros::NodeHandle	p_nh_;
		ros::Subscriber	sub_;
		ros::ServiceClient	srv_info_;
		std::string		topic_;

		FactoryWriter	factory;
		std::unique_ptr<Writer> writer_;

		std::string		filename_;
		bool			is_configured_;
		
		rosneuro_msgs::NeuroFrame	msg_;
		NeuroFrame	frame_;

};

}


#endif
