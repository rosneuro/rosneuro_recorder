#include <gtest/gtest.h>
#include "Writer.hpp"

namespace rosneuro {
    class WriterTest : public Writer {
        public:
            WriterTest(NeuroFrame* frame) : Writer(frame) {}
            ~WriterTest() {}
            bool Setup(void) { return true; }
            bool Open(const std::string& filename) { return true; }
            bool Close(void) { return true; }
            int Write(int nswrite) { return 0; }
            bool AddEvent(int event, double onset, double duration) { return true; }
    };

    class WriterTestSuite : public ::testing::Test {
        public:
            WriterTestSuite() {}
            ~WriterTestSuite() {}
            void SetUp() {
                NeuroFrame* frame = new NeuroFrame();
                writer = new WriterTest(frame);
            }
            void TearDown() {
                delete writer;
            }
            Writer* writer;
        };

    TEST_F(WriterTestSuite, TestGetName) {
        EXPECT_EQ(writer->GetName(), "undefined");
        EXPECT_NE(writer->frame_, nullptr);
    }

    TEST_F(WriterTestSuite, TestWho) {
        testing::internal::CaptureStdout();
        writer->Who();
        std::string output = testing::internal::GetCapturedStdout();
        EXPECT_EQ(output, "[undefined] - undefined writer\n");
    }
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}