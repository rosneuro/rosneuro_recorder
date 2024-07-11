#ifndef ROSNEURO_RECORDER_DUMMYWRITER_HPP
#define ROSNEURO_RECORDER_DUMMYWRITER_HPP

#include "rosneuro_recorder/Writer.hpp"
#include "rosneuro_data/NeuroData.hpp"
#include <gtest/gtest_prod.h>

namespace rosneuro {
    class DummyWriter : public Writer {
        public:
            DummyWriter(NeuroFrame* frame);
            virtual ~DummyWriter(void);

            bool Setup(void);
            bool Open(const std::string& filename);
            bool Close(void);
            int Write(int nswrite);
            bool AddEvent(int event, double onset, double duration);

        private:
            FRIEND_TEST(DummyWriterTestSuite, TestGetName);
    };
}

#endif
