#ifndef ROSNEURO_RECORDER_WRITER_HPP
#define ROSNEURO_RECORDER_WRITER_HPP

#include <string>
#include <errno.h>
#include <string.h>
#include <iostream>
#include <cstring>

#include "rosneuro_data/NeuroData.hpp"

namespace rosneuro {


class Writer {

	public:
		Writer(NeuroFrame* frame);
		virtual ~Writer(void);

		virtual bool Setup(void) = 0;
		virtual bool Open(const std::string& filename) = 0;
		virtual bool Close(void) = 0;
		virtual int Write(int nswrite) = 0;


		virtual bool AddEvent(int event, double onset, double duration) = 0;
		
		virtual std::string GetName(void);
		virtual void Who(void);
	
	protected:
		std::string	name_;
		NeuroFrame*	frame_;


};


}


#endif
