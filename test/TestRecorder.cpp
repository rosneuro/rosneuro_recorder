#include <gtest/gtest.h>
#include <fstream>
#include "Recorder.hpp"
#include <thread>

namespace rosneuro {
    bool getAcquisitionInfoCallback(rosneuro_msgs::GetAcquisitionInfo::Request& req,
                                    rosneuro_msgs::GetAcquisitionInfo::Response& res) {
            return true;
    }

    class RecorderTestSuite : public ::testing::Test {
        public:
            RecorderTestSuite() {}
            ~RecorderTestSuite() {}
            void SetUp(void) { recorder = new Recorder(); }
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
            Recorder* recorder;
    };

    TEST_F(RecorderTestSuite, TestConstructor) {
        EXPECT_EQ(recorder->topic_data_, "neurodata");
        EXPECT_EQ(recorder->topic_evt_, "/events/bus");
        EXPECT_EQ(recorder->is_frame_set_, false);
        EXPECT_EQ(recorder->autostart_, false);
        EXPECT_EQ(recorder->firstdata_, false);
        EXPECT_EQ(recorder->state_, Recorder::IS_IDLE);
    }

    TEST_F(RecorderTestSuite, TestGetFilename) {
        std::string filename = recorder->get_filename("Subject", "Modality", "Task", "Extra");
        std::string empty_filename = recorder->get_filename("", "", "", "");
        std::string datetime = recorder->get_datetime();
        std::string expected = "Subject." + datetime + ".Modality.Task.Extra.gdf";
        std::string expected_empty = "UNKNOWN." + datetime + ".neuromodality.neurotask.gdf";
        EXPECT_EQ(filename, expected);
        EXPECT_EQ(empty_filename, expected_empty);
    }

    TEST_F(RecorderTestSuite, TestOnWriter) {
        std::string file_path = GetCurrentDir();
        std::string file_name = "test.xdf";
        ros::param::set("~filepath", file_path);
        ros::param::set("~filename", file_name);

        recorder->configure();

        recorder->state_ = Recorder::IS_WAITING;
        ASSERT_EQ(recorder->on_writer_waiting(), Recorder::IS_STARTING);
        ASSERT_EQ(recorder->is_frame_set_, true);

        recorder->state_ = Recorder::IS_STARTING;
        recorder->autostart_ = true;
        ASSERT_EQ(recorder->on_writer_idle(), Recorder::IS_WAITING);

        recorder->autostart_ = false;
        ASSERT_EQ(recorder->on_writer_idle(), Recorder::IS_IDLE);

        recorder->state_ = Recorder::IS_READY;
        ASSERT_EQ(recorder->on_writer_starting(), Recorder::IS_READY);

        recorder->state_ = Recorder::IS_STARTING;
        recorder->is_frame_set_ = false;
        ASSERT_EQ(recorder->on_writer_starting(), Recorder::IS_WAITING);

        recorder->is_frame_set_ = true;

        ASSERT_EQ(recorder->on_writer_starting(), Recorder::IS_READY);
        ASSERT_EQ(recorder->on_writer_starting(), Recorder::IS_QUIT);

        std_srvs::Empty::Request req;
        std_srvs::Empty::Response res;
        ASSERT_TRUE(recorder->on_request_quit(req, res));

        DeleteFile(file_path + "/" + file_name);
    }

    TEST_F(RecorderTestSuite, TestOnRequestRecord) {
        for (unsigned int i = 0; i < 5; i++) {
            recorder->state_ = i;
            std_srvs::Empty::Request req;
            std_srvs::Empty::Response res;
            ASSERT_TRUE(recorder->on_request_record(req, res));
        }
    }
}

int main(int argc, char **argv) {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal);
    ros::init(argc, argv, "test_recorder");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("/acquisition/get_info", rosneuro::getAcquisitionInfoCallback);

    std::thread spin_thread([&]() { ros::spin(); });

    testing::InitGoogleTest(&argc, argv);
    int done = RUN_ALL_TESTS();
    ros::shutdown();
    spin_thread.join();
    return done;
}