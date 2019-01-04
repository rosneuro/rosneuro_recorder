#ifndef ROSNEURO_RECORDER_HPP
#define ROSNEURO_RECORDER_HPP

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include "rosneuro_recorder/FactoryWriter.hpp"
#include "rosneuro_data/NeuroData.hpp"
#include "rosneuro_data/NeuroDataTools.hpp"
#include "rosneuro_msgs/NeuroEvent.h"
#include "rosneuro_msgs/NeuroFrame.h"
#include "rosneuro_msgs/NeuroDataInfo.h"
#include "rosneuro_msgs/GetAcquisitionInfo.h"

namespace rosneuro {

class Recorder {
	public:
		Recorder(void);
		virtual ~Recorder(void);

		bool configure(void);

		bool Run(void);


	public:
		enum {IS_IDLE, IS_WAITING, IS_STARTING, IS_READY, IS_QUIT};

	private:
		void on_received_data(const rosneuro_msgs::NeuroFrame& msg);
		void on_received_event(const rosneuro_msgs::NeuroEvent& msg);

		unsigned int on_writer_idle(void);
		unsigned int on_writer_waiting(void);
		unsigned int on_writer_starting(void);

		bool on_request_record(std_srvs::Empty::Request& req,
							   std_srvs::Empty::Response& res);
		bool on_request_quit(std_srvs::Empty::Request& req,
							 std_srvs::Empty::Response& res);

		std::string get_datetime(void);

	private:
		ros::NodeHandle		nh_;
		ros::NodeHandle		p_nh_;
		ros::Subscriber		sub_data_;
		ros::Subscriber		sub_evt_;
		ros::ServiceServer	srv_record_;
		ros::ServiceServer	srv_quit_;
		ros::ServiceClient	srv_info_;
		std::string			topic_data_;
		std::string			topic_evt_;
		unsigned int		state_;

		double				starttime_;

		FactoryWriter			factory_;
		std::unique_ptr<Writer> writer_;

		std::string		filename_;
		bool			autostart_;
		bool			is_frame_set_;
		
		NeuroFrame					frame_;
		rosneuro_msgs::NeuroFrame	msg_;

};

}


#endif
