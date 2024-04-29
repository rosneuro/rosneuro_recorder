#ifndef ROSNEURO_RECORDER_CPP
#define ROSNEURO_RECORDER_CPP

#include "rosneuro_recorder/Recorder.hpp"

namespace rosneuro {
    Recorder::Recorder(void) : p_nh_("~") {
        this->topic_data_	 = "neurodata";
        this->topic_evt_	 = "/events/bus";
        this->is_frame_set_  = false;
        this->autostart_	 = false;
        this->firstdata_	 = false;
        this->state_		 = Recorder::IS_IDLE;
    }

    Recorder::~Recorder(void) {
        this->writer_->Close();
    }

    bool Recorder::configure(void) {
        unsigned int write_type = WriterType::XDFWRT;
        const char* default_path;
        std::string filepath, subject, task, modality, extra, default_filename;

        this->writer_ = this->factory_.createWriter(&(this->frame_), write_type);

        if((default_path = std::getenv("ROSNEURO_DATA")) == nullptr) {
            default_path = std::getenv("HOME");
        }
        ros::param::param<std::string>("~filepath", filepath, std::string(default_path));

        ros::param::get("/protocol/subject",  subject);
        ros::param::get("/protocol/modality", modality);
        ros::param::get("/protocol/task",	  task);
        ros::param::get("/protocol/extra",    extra);

        default_filename = get_filename(subject, modality, task, extra);
        ros::param::param<std::string>("~filename", this->filename_, default_filename);
        this->filename_ = filepath + "/" + this->filename_;
        ROS_INFO("Data will be recorded in: %s", this->filename_.c_str());

        ros::param::param("~autostart", this->autostart_, false);

        this->sub_data_ = this->nh_.subscribe(this->topic_data_, 1000, &Recorder::on_received_data, this);
        this->sub_evt_  = this->nh_.subscribe(this->topic_evt_, 1000, &Recorder::on_received_event, this);
        this->srv_info_   = this->nh_.serviceClient<rosneuro_msgs::GetAcquisitionInfo>("/acquisition/get_info");
        this->srv_record_ = this->p_nh_.advertiseService("record", &Recorder::on_request_record, this);
        this->srv_quit_   = this->p_nh_.advertiseService("quit",  &Recorder::on_request_quit, this);

        return true;
    }

    std::string Recorder::get_filename(std::string subject, std::string modality,
                                       std::string task, std::string extra) {
        const std::string osubject = subject.empty() ? "UNKNOWN." : subject + ".";
        const std::string ctime = this->get_datetime() + ".";
        const std::string omodality = modality.empty() ? "neuromodality." : modality + ".";
        const std::string otask = task.empty() ? "neurotask." : task + ".";
        const std::string oextra = extra.empty() ? "" : extra + ".";

        return osubject + ctime + omodality + otask + oextra + "gdf";
    }

    std::string Recorder::get_datetime(void) {
        time_t raw_time;
        struct tm * time_info;
        char buffer [80];

        time (&raw_time);
        time_info = localtime (&raw_time);

        strftime (buffer,80,"%Y%m%d.%H%M%S",time_info);

        return std::string(buffer);
    }

    void Recorder::on_received_data(const rosneuro_msgs::NeuroFrame& msg) {
        size_t gsize = msg.eeg.info.nsamples;

        if(this->state_ != Recorder::IS_READY) {
            return;
        }

        if(!firstdata_) {
            this->starttime_ = ros::Time::now().toSec() - (float)msg.eeg.info.nsamples/(float)msg.sr;
            firstdata_ = true;
        }
        ROS_WARN_ONCE("First NeuroFrame received. The recording started. Message id number: %d", msg.header.seq);

        NeuroDataTools::ToNeuroFrame(msg, this->frame_);
        int wsize = this->writer_->Write(gsize);

        if(wsize == -1) {
            ROS_ERROR("Failed writing on file");
        } else if (wsize != gsize) {
            ROS_WARN("Not all data has been written");
        }
    }

    void Recorder::on_received_event(const rosneuro_msgs::NeuroEvent& msg) {
        if(this->state_ != Recorder::IS_READY) {
            return;
        }

        double onset = msg.header.stamp.toSec() - this->starttime_;
        if(!this->writer_->AddEvent(msg.event, onset, msg.duration)) {
            ROS_ERROR("Cannot add event %d: %s", msg.event, strerror(errno));
            return;
        }

        ROS_INFO("Added event %d at %fs (duration=%f)", msg.event, onset, msg.duration);
    }

    bool Recorder::Run(void) {
        bool quit = false;
        ros::Rate r(8192);

        this->configure();
        ROS_INFO("Recorder successfully configured");

        ROS_INFO("Recorder started");
        while(ros::ok() && !quit) {
            ros::spinOnce();
            r.sleep();
            switch(this->state_) {
                case Recorder::IS_IDLE:
                    this->state_ = this->on_writer_idle();
                    break;
                case Recorder::IS_WAITING:
                    this->state_ = this->on_writer_waiting();
                    break;
                case Recorder::IS_STARTING:
                    this->state_ = this->on_writer_starting();
                    break;
                case Recorder::IS_READY:
                    break;
                case Recorder::IS_QUIT:
                    quit = true;
                    break;
                default:
                    break;
            }
        }
        ROS_INFO("Recorder closed");
        return true;
    }

    unsigned int Recorder::on_writer_idle(void) {
        if(!this->autostart_) {
            ROS_WARN_ONCE("Writer is idle. Waiting for start");
            return Recorder::IS_IDLE;
        }

        ROS_INFO("Writer is waiting for data info from acquisition");
        return Recorder::IS_WAITING;
    }

    unsigned int Recorder::on_writer_waiting(void) {
        rosneuro_msgs::GetAcquisitionInfo info;
        if (!this->srv_info_.call(info)) {
            ROS_WARN_THROTTLE(5.0f, "Waiting for data info from acquisition...");
            return Recorder::IS_WAITING;
        }

        if(!NeuroDataTools::ConfigureNeuroFrame(info.response.frame, this->frame_)) {
            ROS_ERROR("Cannot configure frame");
            return Recorder::IS_QUIT;
        }

        this->frame_.eeg.reserve(this->frame_.eeg.nsamples(), this->frame_.eeg.nchannels());
        this->frame_.exg.reserve(this->frame_.exg.nsamples(), this->frame_.exg.nchannels());
        this->frame_.tri.reserve(this->frame_.tri.nsamples(), this->frame_.tri.nchannels());

        this->frame_.eeg.dump();
        this->frame_.exg.dump();
        this->frame_.tri.dump();

        this->is_frame_set_ = true;

        return Recorder::IS_STARTING;
    }

    unsigned int Recorder::on_writer_starting(void) {
        if(this->state_ == Recorder::IS_READY)
            return Recorder::IS_READY;

        if(!this->is_frame_set_) {
            ROS_WARN("NeuroFrame is not set yet. Waiting for data configuration");
            return Recorder::IS_WAITING;
        }

        if(!this->writer_->Open(this->filename_)) {
            ROS_ERROR("Cannot open the file: %s", this->filename_.c_str());
            return Recorder::IS_QUIT;
        }
        ROS_INFO("File open: %s", this->filename_.c_str());

        if(!this->writer_->Setup()) {
            ROS_ERROR("Cannot setup the writer");
            return Recorder::IS_QUIT;
        }
        ROS_INFO("Writer correctly setup");
        return Recorder::IS_READY;
    }

    bool Recorder::on_request_record(std_srvs::Empty::Request& req,
                                     std_srvs::Empty::Response& res) {
        if(this->state_ == Recorder::IS_READY) {
            ROS_INFO("Recorder is already recording");
        }
        else if(this->state_ == Recorder::IS_WAITING) {
            ROS_INFO("Recorder is waiting for data info from acquisition");
        }
        else if(this->state_ == Recorder::IS_STARTING) {
            ROS_INFO("Recorder is already starting");
        }
        else {
            ROS_INFO("Writer is waiting for data info from acquisition");
            this->state_ = Recorder::IS_WAITING;
        }
        return true;
    }

    bool Recorder::on_request_quit(std_srvs::Empty::Request& req,
                                   std_srvs::Empty::Response& res) {
        ROS_WARN("Requested recorder to quit");
        if(!this->writer_->Close()) {
            ROS_ERROR("Cannot close the recorder");
            this->state_ = Recorder::IS_QUIT;
        }
        ROS_INFO("Recorder correctly closed");
        this->state_ = Recorder::IS_QUIT;

        return true;
    }
}

#endif
