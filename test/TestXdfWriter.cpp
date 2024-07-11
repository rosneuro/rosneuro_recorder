#include <gtest/gtest.h>
#include <fstream>
#include "XDFWriter.hpp"

namespace rosneuro {
    class XDFWriterTestSuite : public ::testing::Test {
        public:
            XDFWriterTestSuite() {}
            ~XDFWriterTestSuite() {}

            void SetUp(void) {
                frame = new NeuroFrame();
                writer = new XDFWriter(frame);
            }

            void TearDown(void) {
                delete writer;
                delete frame;
            }

            std::string GetCurrentDir(void) {
                char cwd[1024];
                if (getcwd(cwd, sizeof(cwd)) != NULL)
                    return std::string(cwd);
                throw std::runtime_error("Error getting current directory");
            }

            void DeleteFile(const std::string& path) {
                std::ifstream file;
                file.open(path);
                ASSERT_TRUE(file);
                std::remove(path.c_str());
            }

            std::string dir = GetCurrentDir();
            XDFWriter* writer;
            NeuroFrame* frame;
    };

    TEST_F(XDFWriterTestSuite, TestConstructor) {
        EXPECT_EQ(writer->name_, "xdfwriter");
        EXPECT_EQ(writer->file_, nullptr);
        EXPECT_EQ(writer->strides_, nullptr);
    }

    TEST_F(XDFWriterTestSuite, TestSetupSuccess) {
        std::string path = dir + "TestSetupSuccess.gdf";

        writer->Open(path);
        ASSERT_TRUE(writer->Setup());
        ASSERT_NE(writer->strides_, nullptr);

        DeleteFile(path);
    }

    TEST_F(XDFWriterTestSuite, TestSetupFailure) {
        EXPECT_EQ(writer->Setup(), false);
    }

    TEST_F(XDFWriterTestSuite, TestOpenFailure) {
        std::string path = dir + "TestOpenFailure.gdf";

        ASSERT_TRUE(writer->Open(path));
        ASSERT_FALSE(writer->Open(path));

        DeleteFile(path);
    }

    TEST_F(XDFWriterTestSuite, TestCloseSuccess){
        writer->file_ = nullptr;
        ASSERT_TRUE(writer->Close());

        std::string path = dir + "TestCloseSuccess.gdf";
        writer->Open(path);

        ASSERT_TRUE(writer->Close());

        DeleteFile(path);
    }

    TEST_F(XDFWriterTestSuite, TestWriteSuccess){
        std::string path = dir + "TestWriteSuccess.gdf";
        writer->Open(path);
        writer->Setup();

        ASSERT_EQ(writer->Write(1), 1);

        DeleteFile(path);
    }

    TEST_F(XDFWriterTestSuite, TestWriteFailure){
        writer->file_ = nullptr;
        ASSERT_EQ(writer->Write(1), -1);
    }

    TEST_F(XDFWriterTestSuite, TestAddEventSuccess){
        std::string path = dir + "TestAddEventSuccess.gdf";
        writer->Open(path);
        writer->Setup();

        int event = 1;
        double onset = 0.0;
        double duration = 1.0;
        writer->AddEvent(event, onset, duration);

        ASSERT_TRUE(writer->AddEvent(event, onset, duration));
        DeleteFile(path);
    }

    TEST_F(XDFWriterTestSuite, TestAddEventFailure){
        int event = 1;
        double onset = 0.0;
        double duration = 1.0;
        ASSERT_FALSE(writer->AddEvent(event, onset, duration));
    }
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}