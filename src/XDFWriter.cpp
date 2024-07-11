#ifndef ROSNEURO_RECORDER_XDFWRITER_CPP
#define ROSNEURO_RECORDER_XDFWRITER_CPP

#include "rosneuro_recorder/XDFWriter.hpp"

namespace rosneuro {
    XDFWriter::XDFWriter(NeuroFrame* frame) : Writer(frame) {
        this->name_	   = "xdfwriter";
        this->file_    = nullptr;
        this->strides_ = nullptr;
    }

    XDFWriter::~XDFWriter(void) {
        if(this->strides_ != nullptr)
            free(this->strides_);
        this->strides_ = nullptr;
    }

    bool XDFWriter::Setup(void) {
        if(xdf_set_conf(this->file_,
                        XDF_F_REC_DURATION, 1.0,
                        XDF_F_REC_NSAMPLE, this->frame_->sr,
                        XDF_NOF) == -1) {
            std::cerr<<"Cannot setup header: " << strerror(errno) <<std::endl;
            return false;
        }

        if(!this->setup_xdf_group(this->frame_->eeg.info(), 0)) {
            std::cerr<<"Cannot setup EEG group"<<std::endl;
            return false;
        }

        if(!this->setup_xdf_group(this->frame_->exg.info(), 1)) {
            std::cerr<<"Cannot setup EXG group"<<std::endl;
            return false;
        }

        if(!this->setup_xdf_group(this->frame_->tri.info(), 2)) {
            std::cerr<<"Cannot setup TRI group"<<std::endl;
            return false;
        }

        this->strides_ = (size_t*)malloc(3 * sizeof(size_t));
        memset(this->strides_, 0, 3 * sizeof(size_t));

        this->strides_[0] = this->frame_->eeg.nchannels()*sizeof(float);
        this->strides_[1] = this->frame_->exg.nchannels()*sizeof(float);
        this->strides_[2] = this->frame_->tri.nchannels()*sizeof(int32_t);

        if(xdf_define_arrays(this->file_, 3, this->strides_) == -1) {
            std::cerr<<"Cannot define strides:" << strerror(errno) <<std::endl;
            return false;
        }

        if(xdf_prepare_transfer(this->file_) == -1) {
            std::cerr<<"Cannot prepare transfer: " << strerror(errno) <<std::endl;
            return false;
        }
        return true;
    }

    bool XDFWriter::setup_xdf_group(NeuroDataInfo* info, unsigned int index) {
        struct xdfch* ch;
        int nch = 0;

        if(info == nullptr) {
            return false;
        }

        xdf_set_conf(this->file_,
                     XDF_CF_ARRINDEX, index,
                     XDF_CF_ARROFFSET, 0,
                     XDF_CF_ARRDIGITAL, 0,
                     XDF_CF_ARRTYPE, info->isint ? XDFINT32 : XDFFLOAT,
                     XDF_CF_STOTYPE, xdf_closest_type(this->file_, XDFFLOAT),
                     XDF_CF_PMIN, info->minmax[0],
                     XDF_CF_PMAX, info->minmax[1],
                     XDF_CF_TRANSDUCTER, info->transducter,
                     XDF_CF_PREFILTERING, info->prefiltering,
                     XDF_CF_UNIT, info->unit,
                     XDF_NOF);

        for(auto it = info->labels.begin(); it != info->labels.end(); ++it) {
            if((ch = xdf_add_channel(this->file_, (*it).c_str())) == NULL) {
                return false;
            }
        }

        return true;
    }

    bool XDFWriter::Open(const std::string& filename) {
        xdffiletype type = this->get_filetype(filename);

        this->file_ = xdf_open(filename.c_str(), XDF_WRITE, type);
        if(this->file_ == nullptr) {
            std::cerr<<"Cannot open file: " << strerror(errno) << std::endl;
            return false;
        }

        return true;
    }

    bool XDFWriter::Close(void) {
        if(this->file_ == nullptr) {
            return true;
        }

        if(xdf_close(this->file_) == -1) {
            std::cerr<<"Cannot close file: " << strerror(errno) << std::endl;
            return false;
        }

        this->file_ = nullptr;
        return true;
    }

    int XDFWriter::Write(int nswrite) {
        if(this->file_ == nullptr) {
            return -1;
        }
        return xdf_write(this->file_, nswrite,
                         this->frame_->eeg.data(),
                         this->frame_->exg.data(),
                         this->frame_->tri.data());
    }

    xdffiletype XDFWriter::get_filetype(const std::string& filename) {
        enum xdffiletype type;

        if(filename.find(".bdf") != std::string::npos) {
            type = XDF_BDF;
        } else if(filename.find(".gdf") != std::string::npos) {
            type = XDF_GDF2;
        } else {
            type = XDF_GDF2;
        }

        return type;
    }

    bool XDFWriter::AddEvent(int event, double onset, double duration) {
        int event_type = xdf_add_evttype(this->file_, event, "");

        if(event_type == -1) {
            return false;
        }

        if(xdf_add_event(this->file_, event_type, onset, duration) == -1) {
            return false;
        }

        return true;
    }
}

#endif
