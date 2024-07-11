#ifndef ROSNEURO_RECORDER_XDFWRITER_HPP
#define ROSNEURO_RECORDER_XDFWRITER_HPP

#include <xdfio.h>
#include "rosneuro_recorder/Writer.hpp"
#include "rosneuro_data/NeuroData.hpp"
#include <gtest/gtest_prod.h>

namespace rosneuro {
    class XDFWriter : public Writer {
        public:
            XDFWriter(NeuroFrame* frame);
            virtual ~XDFWriter(void);

            bool Setup(void);
            bool Open(const std::string& filename);
            bool Close(void);
            int Write(int nswrite);

            bool AddEvent(int event, double onset, double duration);

        private:
            bool setup_xdf_group(NeuroDataInfo* info, unsigned int index);
            xdffiletype get_filetype(const std::string& filename);

        protected:
            struct xdf*	file_;
            size_t*	strides_;

            FRIEND_TEST(XDFWriterTestSuite, TestConstructor);
            FRIEND_TEST(XDFWriterTestSuite, TestSetupSuccess);
            FRIEND_TEST(XDFWriterTestSuite, TestCloseSuccess);
            FRIEND_TEST(XDFWriterTestSuite, TestWriteSuccess);
            FRIEND_TEST(XDFWriterTestSuite, TestWriteFailure);

    };
}


#endif
