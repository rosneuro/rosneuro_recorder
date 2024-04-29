#ifndef ROSNEURO_RECORDER_FACTORY_WRITER_HPP
#define ROSNEURO_RECORDER_FACTORY_WRITER_HPP

#include <memory>
#include "rosneuro_data/NeuroData.hpp"
#include "rosneuro_recorder/Writer.hpp"
#include "rosneuro_recorder/XDFWriter.hpp"
#include "rosneuro_recorder/DummyWriter.hpp"

namespace rosneuro {
    enum WriterType {XDFWRT, DUMMYWRT};
    class FactoryWriter {
        public:
            std::unique_ptr<Writer> createWriter(NeuroFrame* frame, unsigned int type = WriterType::XDFWRT);

    };
}

#endif
