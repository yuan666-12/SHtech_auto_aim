#include "MvCameraControl.h"
#include "pthread.h"
#include <opencv2/opencv.hpp>
#include "../cam_wrapper.hpp"

class HikCamWrapper : public WrapperHead {
  public:
    enum class BayerPattern { AUTO = 0, RG, GB };

    cv::Mat img;

    HikCamWrapper(int dev_num = 0, BayerPattern bayer_pattern = BayerPattern::AUTO);
    ~HikCamWrapper();

    bool init(bool debug = false);

    bool setBrightness(int brightness);
    int getFps();
    void setBayerPattern(BayerPattern bayer_pattern);

    bool read(cv::Mat &src, bool debug = false);

    bool close(bool debug = false);

    cv::Size getSize();
    std::chrono::steady_clock::time_point frame_received_at() const override { return frame_received_at_; }
    uint64_t frame_number() const override { return frame_number_; }
    int64_t host_timestamp() const override { return host_timestamp_; }
    uint64_t device_timestamp() const override { return device_timestamp_; }

  private:
    std::chrono::steady_clock::time_point frame_received_at_{};
    uint64_t frame_number_ = 0, device_timestamp_ = 0;
    int64_t host_timestamp_ = 0;
    double exposure;
    double gain;
    void *cam_handle;
    bool device_open_ = false;
    bool grabbing_ = false;
    unsigned int dev_num;
    unsigned int fail_cnt;

    unsigned char *pData;
    unsigned char *pDataForBGR;
    int nDataSize;
    BayerPattern configured_bayer_pattern_;
    BayerPattern bayer_pattern_;

    MV_IMAGE_BASIC_INFO stCamInfo{};
    MV_FRAME_OUT stOutFrame = {0};
    MVCC_INTVALUE stParam;

    int open_device(bool debug);
    int configure_stream(bool debug);
    int start_stream(bool debug);

    bool updateBayerPatternFromPixelType(unsigned int pixel_type);
    int getCvBayerConversionCode() const;
    bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo);
};
