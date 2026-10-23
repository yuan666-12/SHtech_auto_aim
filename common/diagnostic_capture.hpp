#pragma once
#include "diagnostic_session.hpp"
#include "datatype.hpp"
#include <condition_variable>
#include <cstdio>
#include <fstream>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

// Optional sampled JPEGs, independent of the existing calibration/DRY_RUN capture.
namespace diagnostic_capture {
class Recorder {
    struct Job {
        cv::Mat image;
        long long image_ns=0, steady_ns=0, wall_ns=0;
        uint64_t camera_frame=0, id=0;
        int source_width=0,source_height=0;
        double yaw=0,pitch=0,image_age_ms=0;
        bool attitude_valid=false;
    };
    diagnostic_session::Session& session_;
    std::filesystem::path directory_;
    std::ofstream manifest_;
    std::mutex mutex_;
    std::condition_variable condition_;
    std::optional<Job> pending_;
    bool stop_=false;
    std::thread worker_;
    diagnostic_session::Clock::time_point next_{};
    uint64_t sequence_=0;
    std::atomic<uint64_t> dropped_{0},saved_{0};
    void work() noexcept {
        for(;;) {
            Job job;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                condition_.wait(lock,[&]{return stop_ || pending_.has_value();});
                if(stop_ && !pending_)break;
                job=std::move(*pending_);pending_.reset();
            }
            try {
                if(!session_.active()){++dropped_;continue;}
                std::vector<uchar> encoded;
                if(!cv::imencode(".jpg",job.image,encoded,
                    {cv::IMWRITE_JPEG_QUALITY,diagnostic_session::config().jpeg_quality})) {
                    ++dropped_;continue;
                }
                const std::string filename="frame_"+std::to_string(job.id)+".jpg";
                std::ostringstream row;row<<std::setprecision(17)
                    <<job.id<<','<<filename<<','<<job.wall_ns<<','<<job.steady_ns<<','
                    <<job.image_ns<<','<<job.camera_frame<<','<<job.source_width<<','
                    <<job.source_height<<','<<job.image.cols<<','<<job.image.rows<<','
                    <<job.yaw<<','<<job.pitch<<','<<job.attitude_valid<<','<<job.image_age_ms
                    <<','<<dropped_.load()<<'\n';
                const auto text=row.str();
                if(!session_.reserve(encoded.size()+text.size())){++dropped_;continue;}
                std::ofstream file(directory_/filename,std::ios::binary);
                file.write(reinterpret_cast<const char*>(encoded.data()),encoded.size());
                file.close();
                if(!file){++dropped_;continue;}
                manifest_<<text;++saved_;
                if(saved_.load()%10==0)manifest_.flush();
            } catch(...) {++dropped_;}
        }
        manifest_.flush();
    }
public:
    explicit Recorder(const char* project):session_(diagnostic_session::Session::get(project)) {
        if(!session_.active())return;
        directory_=session_.directory()/"frames";
        std::error_code error;std::filesystem::create_directories(directory_,error);
        if(error)return;
        manifest_.open(directory_/"frames.csv");
        if(!manifest_)return;
        const std::string header=
            "sequence,file,wall_ns,enqueue_steady_ns,image_ns,camera_frame,source_width,source_height,"
            "saved_width,saved_height,rx_yaw,rx_pitch,attitude_valid,image_age_ms,dropped_before\n";
        if(!session_.reserve(header.size()))return;
        manifest_<<header;manifest_.flush();
        worker_=std::thread([this]{work();});
    }
    ~Recorder() {
        {std::lock_guard<std::mutex> lock(mutex_);stop_=true;}
        condition_.notify_one();if(worker_.joinable())worker_.join();
        if(manifest_)std::cerr<<"[DIAG_CAPTURE] saved="<<saved_<<" dropped="<<dropped_<<'\n';
    }
    void submit(const ThreadDataPack& data) noexcept {
        try {
            if(!worker_.joinable() || !session_.active() || data.frame.empty())return;
            const auto now=diagnostic_session::Clock::now();if(now<next_)return;
            next_=now+std::chrono::duration_cast<diagnostic_session::Clock::duration>(
                std::chrono::duration<double>(1/diagnostic_session::config().fps));
            std::unique_lock<std::mutex> lock(mutex_,std::try_to_lock);
            if(!lock.owns_lock() || pending_){++dropped_;return;}
            Job job;
            const double scale=std::min(1.0,double(diagnostic_session::config().width)/data.frame.cols);
            // Own a resized snapshot; never keep or modify a pipeline image buffer.
            if(scale<1)cv::resize(data.frame,job.image,cv::Size(),scale,scale,cv::INTER_AREA);
            else job.image=data.frame.clone();
            job.id=++sequence_;job.source_width=data.frame.cols;job.source_height=data.frame.rows;
            job.image_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(data.time.time_since_epoch()).count();
            job.steady_ns=diagnostic_session::ns(now);
            job.wall_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            job.camera_frame=data.camera_frame_number;job.yaw=data.attitude.yaw();job.pitch=data.attitude.pitch();
            job.attitude_valid=data.attitude_valid;
            job.image_age_ms=data.time==std::chrono::high_resolution_clock::time_point{} ? NAN :
                std::chrono::duration<double,std::milli>(
                    std::chrono::high_resolution_clock::now()-data.time).count();
            pending_=std::move(job);lock.unlock();condition_.notify_one();
        } catch(...) {++dropped_;}
    }
};
inline void sample(const ThreadDataPack& data,const char* project) noexcept {
    const auto& c=diagnostic_session::config();if(!c.enabled || !c.capture)return;
    try {static Recorder recorder(project);recorder.submit(data);}catch(...){}
}
} // namespace diagnostic_capture
