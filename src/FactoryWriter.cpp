#ifndef ROSNEURO_RECORDER_FACTORY_WRITER_CPP
#define ROSNEURO_RECORDER_FACTORY_WRITER_CPP

#include "rosneuro_recorder/FactoryWriter.hpp"

namespace rosneuro {

std::unique_ptr<Writer> FactoryWriter::createWriter(NeuroFrame* frame, unsigned int type) {

	std::unique_ptr<Writer> wrt;
	switch(type) {
		case WriterType::XDFWRT:
			wrt = std::unique_ptr<XDFWriter>(new XDFWriter(frame));
			break;
		case WriterType::DUMMYWRT:
			wrt = std::unique_ptr<DummyWriter>(new DummyWriter(frame));
			break;
		default:
			printf("[FactoryWriter] - Unknown writer type required: %u\n", type);
			wrt = std::unique_ptr<Writer>(nullptr);
			break;
	}

	return wrt;
}

}

#endif
