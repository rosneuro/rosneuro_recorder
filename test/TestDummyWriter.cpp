#include <gtest/gtest.h>
#include "DummyWriter.hpp"

namespace rosneuro {
    class DummyWriterTestSuite : public ::testing::Test {
        public:
            DummyWriterTestSuite() {}
            ~DummyWriterTestSuite() {}
            void SetUp() {
                NeuroFrame* frame = new NeuroFrame();
                dummy_writer = new DummyWriter(frame);
            }
            void TearDown() { delete dummy_writer; }
            DummyWriter* dummy_writer;
    };

    TEST_F(DummyWriterTestSuite, TestGetName) {
        EXPECT_EQ(dummy_writer->name_, "dummywriter");
        EXPECT_NE(dummy_writer->frame_, nullptr);
    }

    TEST_F(DummyWriterTestSuite, TestSetup) {
        EXPECT_TRUE(dummy_writer->Setup());
    }

    TEST_F(DummyWriterTestSuite, TestOpen) {
        EXPECT_TRUE(dummy_writer->Open(""));
    }

    TEST_F(DummyWriterTestSuite, TestClose) {
        EXPECT_TRUE(dummy_writer->Close());
    }

    TEST_F(DummyWriterTestSuite, TestWrite) {
        EXPECT_EQ(dummy_writer->Write(0), 0);
    }

    TEST_F(DummyWriterTestSuite, TestAddEvent) {
        EXPECT_TRUE(dummy_writer->AddEvent(0, 0.0, 0.0));
    }

    TEST_F(DummyWriterTestSuite, WhoTest) {
        testing::internal::CaptureStdout();
        dummy_writer->Who();
        std::string output = testing::internal::GetCapturedStdout();
        EXPECT_EQ(output, "[dummywriter] - dummywriter writer\n");
    }
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}